import argparse
import json
import math
import re
import sys
import threading
import time

from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, Common_pb2, Session_pb2
from kortex_api.Exceptions.KServerException import KServerException

from pynput import keyboard


TIMEOUT_DURATION = 20

CONTROLLABLE_ARM_STATES = (
    Common_pb2.ARMSTATE_SERVOING_READY,
    Common_pb2.ARMSTATE_SERVOING_MANUALLY_CONTROLLED,
)

# Starting pose, taken from Monitoring_2026-08-20_5-09-26.json and folded into
# the -180..180 range that the joint commands expect. The tool sits at roughly
# x 0.365, y 0.115, z 0.203 metres with the gripper pointing down, which is a
# reasonable spot to look down on small boxes on a table.
DEFAULT_START_JOINTS = [26.966, -20.039, 87.071, -88.829, -61.149, -46.073]


# --------------------------------------------------------------------------
# Reading a starting pose out of a web app Monitoring export
# --------------------------------------------------------------------------
def parse_number(text):
    """Pull the leading number out of a string like '339.961 °' or '0.365 m'."""
    match = re.search(r"-?\d+(\.\d+)?", str(text))
    return float(match.group()) if match else None


def wrap_180(angle):
    """Fold an angle into -180..180, so 339.961 becomes -20.039."""
    return (angle + 180.0) % 360.0 - 180.0


def load_start_joints(path):
    """
    Read the six joint angles out of a Monitoring export from the web app.

    That file stores the actuator table as a flat list: a run of column headers,
    then a row header such as 'position' followed by one value per joint. So we
    look for the position row and take the six values that come after it.
    """
    with open(path) as handle:
        sections = json.load(handle)

    for section in sections:
        if section.get("title") != "actuators":
            continue
        entries = section.get("data", [])
        for index, entry in enumerate(entries):
            if entry.get("isHeaderRow") and entry.get("title") == "position":
                values = []
                for item in entries[index + 1:]:
                    if "value" not in item:
                        break
                    values.append(wrap_180(parse_number(item["value"])))
                if len(values) >= 6:
                    return values[:6]
    raise ValueError("Could not find joint positions in {}".format(path))


# --------------------------------------------------------------------------
# Connection helper, the same pattern as examples/utilities.py
# --------------------------------------------------------------------------
class DeviceConnection:

    TCP_PORT = 10000

    def __init__(self, ip_address, port=TCP_PORT, credentials=("admin", "admin")):
        self.ip_address = ip_address
        self.port = port
        self.credentials = credentials
        self.session_manager = None
        self.transport = TCPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    def __enter__(self):
        self.transport.connect(self.ip_address, self.port)
        session_info = Session_pb2.CreateSessionInfo()
        session_info.username = self.credentials[0]
        session_info.password = self.credentials[1]
        session_info.session_inactivity_timeout = 10000
        session_info.connection_inactivity_timeout = 2000
        self.session_manager = SessionManager(self.router)
        print("Logging in as", self.credentials[0], "on device", self.ip_address)
        self.session_manager.CreateSession(session_info)
        return self.router

    def __exit__(self, exc_type, exc_value, traceback):
        if self.session_manager is not None:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000
            self.session_manager.CloseSession(router_options)
        self.transport.disconnect()


# --------------------------------------------------------------------------
# Moving to the starting pose
# --------------------------------------------------------------------------
def check_for_end_or_abort(e):
    def check(notification, e=e):
        if notification.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
            e.set()
    return check


def move_to_joint_angles(base, angles):
    """
    Send the arm to a set of joint angles and wait until it gets there.

    This uses PlayJointTrajectory, which plans a smooth motion and reports when
    it is done, in the same way as 102-Movement_high_level/04-send_joint_speeds.py.
    """
    constrained = Base_pb2.ConstrainedJointAngles()
    for joint_id, value in enumerate(angles):
        joint_angle = constrained.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = value

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e), Base_pb2.NotificationOptions()
    )

    print("Moving to the starting pose: "
          + ", ".join("{:.1f}".format(a) for a in angles))
    try:
        base.PlayJointTrajectory(constrained)
    except KServerException as ex:
        base.Unsubscribe(notification_handle)
        print("Could not play the joint trajectory. Error_code:{} , "
              "Sub_error_code:{}".format(ex.get_error_code(),
                                         ex.get_error_sub_code()))
        return False

    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)
    print("Starting pose reached" if finished else "Timeout on the way to the pose")
    return finished


def set_gripper_position(base, value):
    """Send one gripper position, 0.0 fully open to 1.0 fully closed."""
    command = Base_pb2.GripperCommand()
    command.mode = Base_pb2.GRIPPER_POSITION
    finger = command.gripper.finger.add()
    finger.finger_identifier = 1
    finger.value = max(0.0, min(1.0, value))
    try:
        base.SendGripperCommand(command)
    except KServerException:
        pass


# --------------------------------------------------------------------------
# Keyboard
# --------------------------------------------------------------------------
class TargetKeyboard:

    KEY_AXIS = {
        "w": (0, +1.0), "s": (0, -1.0),
        "a": (1, +1.0), "d": (1, -1.0),
        "r": (2, +1.0), "f": (2, -1.0),
    }
    SPECIAL_KEYS = {
        keyboard.Key.up: "up",
        keyboard.Key.down: "down",
        keyboard.Key.left: "left",
        keyboard.Key.right: "right",
        keyboard.Key.page_up: "page_up",
        keyboard.Key.page_down: "page_down",
    }
    # Index 0 is roll about X, index 1 is pitch about Y, index 2 is yaw about Z.
    ANGULAR_KEYS = {
        "page_up": (0, +1.0), "page_down": (0, -1.0),
        "up": (1, +1.0), "down": (1, -1.0),
        "left": (2, +1.0), "right": (2, -1.0),
    }
    GRIPPER_KEYS = {"o": +1.0, "c": -1.0}
    FULL_OPEN_KEY = "1"
    FULL_CLOSE_KEY = "2"
    RETURN_KEY = "h"

    def __init__(self, step_speed=0.06, rot_speed=25.0):
        self.step_speed = step_speed
        self.rot_speed = rot_speed
        self.lock = threading.Lock()
        self.pressed = set()
        self.snap_requested = False
        self.return_requested = False
        self.quit_requested = False
        self.listener = keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release
        )

    def start(self):
        self.listener.start()

    def stop(self):
        self.listener.stop()

    def _on_press(self, key):
        try:
            name = key.char.lower()
        except AttributeError:
            if key == keyboard.Key.esc:
                self.quit_requested = True
                return
            if key == keyboard.Key.space:
                self.snap_requested = True
                return
            name = self.SPECIAL_KEYS.get(key)
            if name is None:
                return
        if name == self.RETURN_KEY:
            self.return_requested = True
        with self.lock:
            self.pressed.add(name)

    def _on_release(self, key):
        try:
            name = key.char.lower()
        except AttributeError:
            name = self.SPECIAL_KEYS.get(key)
            if name is None:
                return
        with self.lock:
            self.pressed.discard(name)

    def _keys(self):
        with self.lock:
            return set(self.pressed)

    def target_velocity(self):
        """How fast the target point should move, in metres per second."""
        vel = [0.0, 0.0, 0.0]
        for name in self._keys():
            if name in self.KEY_AXIS:
                axis, sign = self.KEY_AXIS[name]
                vel[axis] += sign * self.step_speed
        return vel

    def angular_velocity(self):
        """Turn rates for roll, pitch and yaw, in degrees per second."""
        rates = [0.0, 0.0, 0.0]
        for name in self._keys():
            if name in self.ANGULAR_KEYS:
                axis, sign = self.ANGULAR_KEYS[name]
                rates[axis] += sign
        return [max(-1.0, min(1.0, rate)) * self.rot_speed for rate in rates]

    def gripper_direction(self):
        direction = 0.0
        for name in self._keys():
            direction += self.GRIPPER_KEYS.get(name, 0.0)
        return max(-1.0, min(1.0, direction))

    def gripper_jump(self):
        keys = self._keys()
        if self.FULL_OPEN_KEY in keys:
            return 0.0
        if self.FULL_CLOSE_KEY in keys:
            return 1.0
        return None


# --------------------------------------------------------------------------
# Maths and small API helpers
# --------------------------------------------------------------------------
def clamp(value, limit):
    return max(-limit, min(limit, value))


def angle_error_deg(target_deg, current_deg):
    """Shortest angular difference, wrapped into -180..180 degrees."""
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0


def read_gripper_position(base):
    """Gripper opening, 0.0 fully open to 1.0 fully closed, or None if absent."""
    request = Base_pb2.GripperRequest()
    request.mode = Base_pb2.GRIPPER_POSITION
    try:
        measure = base.GetMeasuredGripperMovement(request)
    except KServerException:
        return None
    if not len(measure.finger):
        return None
    return measure.finger[0].value


def compute_ik(base, position, orientation, guess_angles):
    """
    Ask the arm which joint angles put the tool at this pose.

    The guess matters: a six joint arm can reach most poses in more than one
    way, and the solver returns the answer nearest the guess, which is the one
    that does not require the elbow to flip. Returns None if there is no
    solution for this pose.
    """
    ik_data = Base_pb2.IKData()
    ik_data.cartesian_pose.x = position[0]
    ik_data.cartesian_pose.y = position[1]
    ik_data.cartesian_pose.z = position[2]
    ik_data.cartesian_pose.theta_x = orientation[0]
    ik_data.cartesian_pose.theta_y = orientation[1]
    ik_data.cartesian_pose.theta_z = orientation[2]
    for angle in guess_angles:
        joint_angle = ik_data.guess.joint_angles.add()
        joint_angle.value = angle
    try:
        result = base.ComputeInverseKinematics(ik_data)
    except KServerException:
        return None
    return [joint_angle.value for joint_angle in result.joint_angles]


# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
def run(args):
    if args.start_json:
        start_joints = load_start_joints(args.start_json)
        print("Starting pose loaded from", args.start_json)
    else:
        start_joints = list(DEFAULT_START_JOINTS)

    with DeviceConnection(args.ip, credentials=(args.username, args.password)) as router:
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        try:
            base.ClearFaults()
            servoing = Base_pb2.ServoingModeInformation()
            servoing.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            base.SetServoingMode(servoing)
        except KServerException as ex:
            print("Could not set single level servoing. Error_code:{} , "
                  "Sub_error_code:{}".format(ex.get_error_code(),
                                             ex.get_error_sub_code()))
            return 1
        time.sleep(0.2)

        actuator_count = base.GetActuatorCount().count
        if len(start_joints) != actuator_count:
            print("The starting pose has {} angles but the arm has {} joints."
                  .format(len(start_joints), actuator_count))
            return 1

        # Open the gripper before moving, so nothing is trapped in the fingers.
        if not args.start_here:
            set_gripper_position(base, args.start_gripper)
            time.sleep(0.5)
            if not move_to_joint_angles(base, start_joints):
                return 1
        else:
            print("Starting from the arm's current pose")

        # Build the joint speed message once and keep references to its entries.
        joint_speeds = Base_pb2.JointSpeeds()
        speed_entries = []
        for joint_id in range(actuator_count):
            entry = joint_speeds.joint_speeds.add()
            entry.joint_identifier = joint_id
            entry.value = 0.0
            entry.duration = 0
            speed_entries.append(entry)

        def stop_joints():
            for entry in speed_entries:
                entry.value = 0.0
            try:
                base.SendJointSpeedsCommand(joint_speeds)
            except KServerException:
                pass

        fb = base_cyclic.RefreshFeedback()
        target = [fb.base.tool_pose_x, fb.base.tool_pose_y, fb.base.tool_pose_z]
        orientation = [fb.base.tool_pose_theta_x,
                       fb.base.tool_pose_theta_y,
                       fb.base.tool_pose_theta_z]
        joint_target = [fb.actuators[i].position for i in range(actuator_count)]

        gripper_command = Base_pb2.GripperCommand()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        gripper_finger = gripper_command.gripper.finger.add()
        gripper_finger.finger_identifier = 1

        gripper_target = read_gripper_position(base)
        gripper_present = gripper_target is not None
        if not gripper_present:
            print("No gripper detected, so the gripper keys will do nothing.")
            gripper_target = 0.0
        gripper_sent = gripper_target
        gripper_sent_at = 0.0

        kb = TargetKeyboard(step_speed=args.target_speed, rot_speed=args.rot_speed)
        kb.start()

        period = 1.0 / args.rate
        ik_failures = 0
        print("Ready. w/s = X, a/d = Y, r/f = Z. Arrows tilt and turn, "
              "pgup/pgdn roll. o/c = gripper, 1/2 = gripper open/closed, "
              "h = back to start, space = snap, esc = quit.")

        try:
            while not kb.quit_requested:
                loop_start = time.time()

                # Going back to the starting pose is a planned move, so the
                # velocity loop has to stand down while it happens.
                if kb.return_requested:
                    kb.return_requested = False
                    stop_joints()
                    time.sleep(0.2)
                    print("\nReturning to the starting pose")
                    move_to_joint_angles(base, start_joints)
                    fb = base_cyclic.RefreshFeedback()
                    target = [fb.base.tool_pose_x, fb.base.tool_pose_y,
                              fb.base.tool_pose_z]
                    orientation = [fb.base.tool_pose_theta_x,
                                   fb.base.tool_pose_theta_y,
                                   fb.base.tool_pose_theta_z]
                    joint_target = [fb.actuators[i].position
                                    for i in range(actuator_count)]
                    continue

                fb = base_cyclic.RefreshFeedback()

                if fb.base.fault_bank_a or fb.base.fault_bank_b:
                    print("\nArm reported a fault (bank A: {}, bank B: {}). Stopping."
                          .format(fb.base.fault_bank_a, fb.base.fault_bank_b))
                    break
                if fb.base.active_state not in CONTROLLABLE_ARM_STATES:
                    try:
                        state_name = Common_pb2.ArmState.Name(fb.base.active_state)
                    except ValueError:
                        state_name = str(fb.base.active_state)
                    print("\nArm is no longer ready to be controlled ({}). Stopping."
                          .format(state_name))
                    break

                pose = [fb.base.tool_pose_x, fb.base.tool_pose_y, fb.base.tool_pose_z]
                theta = [fb.base.tool_pose_theta_x,
                         fb.base.tool_pose_theta_y,
                         fb.base.tool_pose_theta_z]
                joint_now = [fb.actuators[i].position for i in range(actuator_count)]

                if kb.snap_requested:
                    target = list(pose)
                    orientation = list(theta)
                    joint_target = list(joint_now)
                    kb.snap_requested = False

                # 1. Move the target with the keyboard.
                previous_target = list(target)
                previous_orientation = list(orientation)

                tvel = kb.target_velocity()
                for i in range(3):
                    target[i] += tvel[i] * period

                avel = kb.angular_velocity()
                for i in range(3):
                    orientation[i] = wrap_180(orientation[i] + avel[i] * period)

                # 2. Keep the target inside a safe box around the base.
                target[0] = clamp(target[0], args.workspace)
                target[1] = clamp(target[1], args.workspace)
                target[2] = max(args.min_height, min(args.workspace, target[2]))

                # 3. Do not let the target run far ahead of the tool.
                lead = [target[i] - pose[i] for i in range(3)]
                lead_norm = math.sqrt(sum(v * v for v in lead))
                if lead_norm > args.max_lead:
                    scale = args.max_lead / lead_norm
                    target = [pose[i] + lead[i] * scale for i in range(3)]
                    lead_norm = args.max_lead

                rot_lead = 0.0
                for i in range(3):
                    error = angle_error_deg(orientation[i], theta[i])
                    if abs(error) > args.max_lead_deg:
                        error = clamp(error, args.max_lead_deg)
                        orientation[i] = wrap_180(theta[i] + error)
                    rot_lead = max(rot_lead, abs(error))

                # 4. Inverse kinematics, only when the target actually moved.
                target_moved = (target != previous_target
                                or orientation != previous_orientation)
                if target_moved:
                    solution = compute_ik(base, target, orientation, joint_now)
                    if solution is None:
                        target = previous_target
                        orientation = previous_orientation
                        ik_failures += 1
                    else:
                        biggest_step = max(
                            abs(angle_error_deg(solution[i], joint_now[i]))
                            for i in range(actuator_count))
                        if biggest_step > args.max_joint_step:
                            target = previous_target
                            orientation = previous_orientation
                            ik_failures += 1
                        else:
                            joint_target = solution
                            ik_failures = 0

                # 5. Turn the joint error into a joint speed, one joint at a time.
                for i in range(actuator_count):
                    error = angle_error_deg(joint_target[i], joint_now[i])
                    speed_entries[i].value = clamp(args.kp_joint * error,
                                                   args.max_joint_speed)
                try:
                    base.SendJointSpeedsCommand(joint_speeds)
                except KServerException as ex:
                    print("\nJoint speed command refused. Error_code:{} , "
                          "Sub_error_code:{}".format(ex.get_error_code(),
                                                     ex.get_error_sub_code()))
                    break

                # 6. Gripper. Opening counts down towards 0.0.
                if gripper_present:
                    jump = kb.gripper_jump()
                    if jump is not None:
                        gripper_target = jump
                    else:
                        gripper_target -= (kb.gripper_direction()
                                           * args.gripper_speed * period)
                        gripper_target = max(0.0, min(1.0, gripper_target))

                    now = time.time()
                    if (abs(gripper_target - gripper_sent) > 0.005
                            and (now - gripper_sent_at) > (1.0 / args.gripper_rate)):
                        gripper_finger.value = gripper_target
                        try:
                            base.SendGripperCommand(gripper_command)
                            gripper_sent = gripper_target
                            gripper_sent_at = now
                        except KServerException:
                            pass

                joint_error = max(abs(angle_error_deg(joint_target[i], joint_now[i]))
                                  for i in range(actuator_count))
                sys.stdout.write(
                    "\rtool [{:+.3f} {:+.3f} {:+.3f}]  rpy [{:+6.1f} {:+6.1f} {:+6.1f}]  "
                    "lead {:.3f} m / {:4.1f} deg  joint err {:5.1f} deg  "
                    "grip {:3.0f}%  {}   ".format(
                        *pose, *theta, lead_norm, rot_lead, joint_error,
                        gripper_target * 100.0,
                        "IK blocked" if ik_failures else "          ")
                )
                sys.stdout.flush()

                sleep_left = period - (time.time() - loop_start)
                if sleep_left > 0:
                    time.sleep(sleep_left)

        except KeyboardInterrupt:
            pass
        finally:
            try:
                stop_joints()
                time.sleep(0.1)
                base.Stop()
            except Exception as ex:
                print("\nStop command failed: {}".format(ex))
            kb.stop()
            time.sleep(0.5)
            print("\nStopped.")

    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ip", type=str, default="192.168.1.10",
                        help="IP address of the robot")
    parser.add_argument("-u", "--username", type=str, default="your username")
    parser.add_argument("-p", "--password", type=str, default="your password")
    parser.add_argument("--start-json", type=str, default=None,
                        help="Monitoring export from the web app to read the "
                             "starting joint angles from")
    parser.add_argument("--start-here", action="store_true",
                        help="skip the move and begin from the current pose")
    parser.add_argument("--start-gripper", type=float, default=0.0,
                        help="gripper opening set before the move, 0.0 open to "
                             "1.0 closed")
    parser.add_argument("--rate", type=float, default=25.0,
                        help="control loop rate in Hz")
    parser.add_argument("--kp-joint", type=float, default=2.0,
                        help="joint gain: commanded deg/s per degree of error")
    parser.add_argument("--max-joint-speed", type=float, default=20.0,
                        help="speed limit for any single joint, in deg/s")
    parser.add_argument("--max-joint-step", type=float, default=30.0,
                        help="reject an IK solution that moves any joint more "
                             "than this many degrees in one go")
    parser.add_argument("--target-speed", type=float, default=0.05,
                        help="how fast a held key drags the target, in m/s")
    parser.add_argument("--rot-speed", type=float, default=20.0,
                        help="how fast the arrow and page keys turn the target, "
                             "in deg/s")
    parser.add_argument("--max-lead", type=float, default=0.04,
                        help="how far the target may run ahead of the tool, in m")
    parser.add_argument("--max-lead-deg", type=float, default=12.0,
                        help="how far the target orientation may run ahead, in deg")
    parser.add_argument("--workspace", type=float, default=0.6,
                        help="soft limit on |x|, |y| and z in metres")
    parser.add_argument("--min-height", type=float, default=0.02,
                        help="soft floor for z in metres")
    parser.add_argument("--gripper-speed", type=float, default=0.4,
                        help="how fast a held gripper key moves the opening, in "
                             "fractions of full travel per second")
    parser.add_argument("--gripper-rate", type=float, default=10.0,
                        help="how often gripper positions are sent, in Hz")
    args = parser.parse_args()
    return run(args)


if __name__ == "__main__":
    exit(main())
