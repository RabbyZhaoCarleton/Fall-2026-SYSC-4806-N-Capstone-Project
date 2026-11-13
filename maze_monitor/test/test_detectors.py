import math

from helpers import Feeder, cmd, odom, scan


def wall_ahead(dist):
    return lambda b: dist if abs(b) < 25 else 2.0


def test_pushing_into_wall_is_a_likely_collision():
    f = Feeder()
    for i in range(40):
        t = i * 0.1
        f.feed(t, '/cmd_vel', cmd(v=0.2))
        f.feed(t + 0.01, '/odom', odom(v=0.0))
        f.feed(t + 0.02, '/scan', scan(wall_ahead(0.12)))
    f.finish(4.0)
    blocked = f.events(key='blocked')
    assert len(blocked) == 1
    assert blocked[0].severity == 'error'
    assert blocked[0].category == 'collision'
    assert 0.9 <= blocked[0].t <= 1.3


def test_following_the_command_raises_nothing():
    f = Feeder()
    for i in range(40):
        t = i * 0.1
        f.feed(t, '/cmd_vel', cmd(v=0.2))
        f.feed(t + 0.01, '/odom', odom(x=0.02 * i, v=0.2))
        f.feed(t + 0.02, '/scan', scan(lambda b: 2.0))
    f.finish(4.0)
    assert [e for e in f.session.timeline.events if e.severity != 'info'] == []


def test_close_to_wall_and_contact():
    f = Feeder()
    f.feed(0.0, '/odom', odom())
    f.feed(0.1, '/scan', scan(lambda b: 0.14 if 80 < b < 100 else 2.0))  # 3.5 cm left
    f.feed(0.3, '/scan', scan(lambda b: float('-inf') if 80 < b < 100 else 2.0))
    f.finish(1.5)
    near = f.events(key='near_wall')
    assert near and near[0].severity == 'warning'
    contact = f.events(key='contact')
    assert contact and contact[0].severity == 'error'


def test_wall_vanishing_below_minimum_range_counts_as_contact():
    f = Feeder()
    f.feed(0.0, '/odom', odom())
    f.feed(0.1, '/scan', scan(lambda b: 0.13 if abs(b) < 10 else 2.0))
    f.feed(0.3, '/scan', scan(lambda b: float('inf') if abs(b) < 10 else 2.0))
    f.finish(1.0)
    assert f.events(key='contact')


def test_wheel_slip():
    f = Feeder()
    for i in range(30):
        t = i * 0.1
        f.feed(t, '/cmd_vel', cmd(v=0.2))
        f.feed(t + 0.01, '/odom', odom(x=0.02 * i, v=0.2))  # odometry says moving
        f.feed(t + 0.02, '/scan', scan(wall_ahead(0.5)))   # wall does not get closer
    f.finish(3.0)
    assert f.events(key='slip')


def test_stuck_while_goal_active():
    from action_msgs.msg import GoalStatus, GoalStatusArray
    f = Feeder()
    st = GoalStatusArray()
    gs = GoalStatus()
    gs.status = 2
    gs.goal_info.goal_id.uuid = [1] * 16
    st.status_list = [gs]
    f.feed(0.0, '/navigate_to_pose/_action/status', st)
    for i in range(130):
        f.feed(i * 0.1, '/odom', odom(x=0.001 * i))
    f.finish(13.0)
    stuck = f.events(key='stuck')
    assert stuck and 9.5 <= stuck[0].t <= 11.0


def test_goal_aborted_is_an_error():
    from action_msgs.msg import GoalStatus, GoalStatusArray

    def status(code):
        st = GoalStatusArray()
        gs = GoalStatus()
        gs.status = code
        gs.goal_info.goal_id.uuid = [7] * 16
        st.status_list = [gs]
        return st

    f = Feeder()
    f.feed(0.0, '/navigate_to_pose/_action/status', status(2))
    f.feed(5.0, '/navigate_to_pose/_action/status', status(6))
    f.finish(6.0)
    failed = [e for e in f.events(category='navigation') if e.severity == 'error']
    assert len(failed) == 1 and 'after 5.0 s' in failed[0].title
    assert f.session.timeline.value('nav.goals_failed', 6.0) == 1


def test_localization_jump():
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    def map_odom(x):
        tr = TransformStamped()
        tr.header.frame_id, tr.child_frame_id = 'map', 'odom'
        tr.transform.translation.x = x
        tr.transform.rotation.w = 1.0
        return TFMessage(transforms=[tr])

    f = Feeder()
    f.feed(0.0, '/odom', odom())
    f.feed(0.1, '/tf', map_odom(0.0))
    f.feed(0.2, '/tf', map_odom(0.01))
    f.feed(0.3, '/tf', map_odom(0.30))
    f.finish(1.0)
    jumps = f.events(category='localization')
    assert len(jumps) == 1 and '29 cm' in jumps[0].title


def test_repeated_log_warnings_are_merged():
    from rcl_interfaces.msg import Log
    f = Feeder()
    for i in range(5):
        f.feed(i * 0.5, '/rosout', Log(level=30, name='controller_server',
                                       msg=f'Control loop missed its desired rate of 10.{i}Hz'))
    f.finish(3.0)
    logs = f.events(category='log')
    assert len(logs) == 1 and logs[0].count == 5


def test_oscillation():
    f = Feeder()
    for i in range(40):
        f.feed(i * 0.1, '/cmd_vel', cmd(v=0.05, w=0.5 if (i // 3) % 2 else -0.5))
        f.feed(i * 0.1 + 0.01, '/odom', odom(v=0.05))
    f.finish(4.0)
    assert f.events(key='oscillation')


def test_diagnosis_reports_problem():
    from maze_monitor.analysis.diagnosis import Units, diagnose
    f = Feeder()
    for i in range(30):
        t = i * 0.1
        f.feed(t, '/cmd_vel', cmd(v=0.2))
        f.feed(t + 0.01, '/odom', odom(v=0.0))
        f.feed(t + 0.02, '/scan', scan(wall_ahead(0.12)))
    tl = f.finish(3.0)
    level, headline, lines, problems = diagnose(tl.snapshot(2.5), Units(True))
    assert level == 'error'
    assert 'collision' in headline.lower()
    assert any('told' in line for line in lines)
    assert not math.isnan(len(problems))


def test_two_sources_on_odom_are_detected_and_ignored_for_distance():
    f = Feeder()
    for i in range(20):
        a = odom(x=0.0)
        a.header.stamp.sec = 100 + i // 10
        a.header.stamp.nanosec = (i % 10) * 100_000_000
        f.feed(i * 0.1, '/odom', a)
        b = odom(x=0.07)  # a second, older stream at another position
        b.header.stamp.sec = 98 + i // 10
        b.header.stamp.nanosec = (i % 10) * 100_000_000
        f.feed(i * 0.1 + 0.05, '/odom', b)
    tl = f.finish(2.0)
    two = [e for e in tl.events if e.title == 'Timestamps go backward on /odom']
    assert len(two) == 1 and two[0].count > 10
    assert tl.value('pose.distance', 2.0, 0.0) < 0.01


def test_goal_churn():
    from action_msgs.msg import GoalStatus, GoalStatusArray
    f = Feeder()
    goals = []
    for k in range(8):
        gs = GoalStatus()
        gs.goal_info.goal_id.uuid = [k + 1] * 16
        goals.append(gs)
        for code, dt in ((2, 0.0), (4, 0.3)):
            gs.status = code
            st = GoalStatusArray()
            st.status_list = [g for g in goals]
            f.feed(k * 2.0 + dt, '/navigate_to_pose/_action/status', st)
            gs = GoalStatus(goal_info=gs.goal_info, status=code)
            goals[-1] = gs
    f.finish(20.0)
    churn = [e for e in f.session.timeline.events if e.key == 'goal_churn']
    assert len(churn) == 1 and churn[0].count >= 3
