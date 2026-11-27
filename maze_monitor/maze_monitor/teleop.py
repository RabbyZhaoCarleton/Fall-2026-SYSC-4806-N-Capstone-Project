"""Keyboard driving: which held keys make which velocity command.

No Qt and no ROS in here, so it can be tested on its own. The GUI maps keys
to the four directions and publishes the result (gui/robot_control.py).
"""

FORWARD, BACK, LEFT, RIGHT = 'forward', 'back', 'left', 'right'
SLOW_FACTOR = 0.4


def velocity(held, linear, angular, slow=False):
    """(linear m/s, angular rad/s) for the held directions.

    Opposite directions cancel. Left always turns counterclockwise, also when
    reversing, as in teleop_twist_keyboard.
    """
    k = SLOW_FACTOR if slow else 1.0
    drive = (FORWARD in held) - (BACK in held)
    turn = (LEFT in held) - (RIGHT in held)
    return drive * linear * k, turn * angular * k
