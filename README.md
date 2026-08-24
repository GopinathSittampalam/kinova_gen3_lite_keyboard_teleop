# Keyboard teleoperation for the Kinova Gen3 Lite
Python code to use keyboard keys to teleoperate Kinova Gen3 Lite robot arm.

Before teleoperation begins, the arm goes to a starting pose that you choose, rather than to the factory Home pose. The idea is that you park the arm once in a good spot for picking up small boxes, export the state from the web app's Monitoring page, and from then on every session begins there.

Install
---
First install Kinova Kortex2 Python API and required dependencies following these instructions

https://github.com/Kinovarobotics/Kinova-kortex2_Gen3_G3L/blob/master/api_python/examples/readme.md

Better to create a Conda environment for this API installation

Run
---
Use default setting
_python gen3_lite_teleop.py_

Set robot arm IP address
_python gen3_lite_pick_teleop.py --ip 192.168.1.10_

Controls
--------
    w / s           move target forward / backward   (+X / -X, base frame)
    a / d           move target left / right         (+Y / -Y)
    r / f           move target up / down            (+Z / -Z)
    up / down       tilt the gripper up / down       (pitch, theta Y)
    left / right    turn the gripper left / right    (yaw, theta Z)
    PgUp / PgDn     roll the gripper                 (roll, theta X)
    o / c           open / close the gripper (while held)
    1 / 2           snap the gripper fully open / fully closed
    h               go back to the starting pose
    space           snap the target back to the current tool pose
    esc             stop the arm and quit


Safety
------
The arm will move as soon as the script connects, so make sure the space around it is clear before you run it and keep the e-stop within reach.
