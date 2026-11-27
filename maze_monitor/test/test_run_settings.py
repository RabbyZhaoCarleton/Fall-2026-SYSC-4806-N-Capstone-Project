"""Run settings, the launch command they make, presets, and parameter overrides."""

import os

import yaml

from maze_monitor import param_files, teleop
from maze_monitor.run_settings import PresetStore, RunSettings, parse_extra_args

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def args_of(cmd):
    assert cmd[:4] == ['ros2', 'launch', 'maze_sim', 'maze.launch.py']
    return dict(a.split(':=', 1) for a in cmd[4:])


def test_command_for_each_mode():
    s = RunSettings(autonomy='explore', shutdown_when_done=True, gui=False)
    a = args_of(s.command())
    assert a['explore'] == 'true' and a['nav'] == 'false'
    assert a['shutdown_when_done'] == 'true'
    assert a['gui'] == 'false' and a['rviz'] == 'true'
    assert a['monitor'] == 'false'           # the monitor must not open a second monitor
    assert a['x_pose'] == '0.25' and a['y_pose'] == '-0.25'
    assert 'world' not in a and 'nav_params' not in a

    a = args_of(RunSettings(autonomy='nav').command())
    assert a['nav'] == 'true' and a['explore'] == 'false' and 'shutdown_when_done' not in a
    a = args_of(RunSettings(autonomy='manual').command())
    assert a['nav'] == 'false' and a['explore'] == 'false'


def test_files_and_extra_args():
    s = RunSettings(world='/tmp/w.sdf', extra_args='rviz_config:="" foo:=a\\ b')
    s.params = {'nav': '/tmp/my_nav.yaml'}
    a = args_of(s.command())
    assert a['world'] == '/tmp/w.sdf'
    assert a['nav_params'] == '/tmp/my_nav.yaml'
    assert a['rviz_config'] == '' and a['foo'] == 'a b'
    # files written for the run win over the chosen files
    a = args_of(s.command({'nav': '/runs/x/params/nav2_maze.yaml'}))
    assert a['nav_params'] == '/runs/x/params/nav2_maze.yaml'


def test_extra_args_are_checked():
    assert parse_extra_args('') == []
    assert parse_extra_args('a:=1 b:=x') == [('a', '1'), ('b', 'x')]
    for bad in ('nonsense', 'a=1', 'monitor:=true', 'a:="unclosed'):
        try:
            parse_extra_args(bad)
            raise AssertionError(bad)
        except ValueError:
            pass
    assert RunSettings(extra_args='oops').problems()
    assert RunSettings(world='/no/such/world.sdf').problems()
    assert RunSettings().problems() == []


def test_round_trip_and_tolerant_loading():
    s = RunSettings(autonomy='nav', gui=True, x_pose=1.5, label='wide', notes='n')
    s.overrides = {'explore': {('/**', 'ros__parameters', 'progress_timeout'): 30.0}}
    d = s.to_dict()
    yaml.safe_load(yaml.safe_dump(d))   # storable as YAML (presets) ...
    import json
    json.loads(json.dumps(d))           # ... and JSON (run.json)
    back = RunSettings.from_dict(d)
    assert back == s
    odd = RunSettings.from_dict({'autonomy': 'fly', 'gui': 'true', 'x_pose': 'x', 'unknown': 1})
    assert odd.autonomy == 'explore' and odd.gui is True and odd.x_pose == 0.25


def test_from_launch_args_of_a_terminal_run():
    s = RunSettings.from_launch_args({'explore': 'false', 'nav': 'true', 'gui': 'true',
                                      'ekf': 'false', 'x_pose': '1.0', 'y_pose': '2.0'})
    assert s.autonomy == 'nav' and s.gui and not s.ekf and (s.x_pose, s.y_pose) == (1.0, 2.0)


def test_presets(tmp_path):
    store = PresetStore(str(tmp_path / 'presets'))
    assert store.names() == []
    store.save('Explore, headless', RunSettings(gui=False, rviz=False))
    store.save('teleop', RunSettings(autonomy='manual'))
    assert store.names() == ['Explore, headless', 'teleop']
    assert store.load('Explore, headless').rviz is False
    assert store.load('teleop').autonomy == 'manual'
    store.delete('teleop')
    assert store.names() == ['Explore, headless']


def test_param_values_keep_their_ros_type():
    assert param_files.parse_value('30', 20.0) == 30.0 and isinstance(
        param_files.parse_value('30', 20.0), float)
    assert param_files.parse_value('3', 2) == 3
    assert param_files.parse_value('false', True) is False
    assert param_files.parse_value('scan2', 'scan') == 'scan2'
    assert param_files.parse_value('[1, 2]', [0.5, 0.5]) == [1.0, 2.0]
    for text, like in (('1.5', 2), ('yes please', True), ('abc', 1.0), ('3', [1.0]),
                       ('["a"]', [1.0]), ('true', 1)):
        try:
            param_files.parse_value(text, like)
            raise AssertionError((text, like))
        except ValueError:
            pass


def test_overrides_are_written_to_the_run_folder(tmp_path):
    src = os.path.join(REPO, 'maze_navigation', 'config', 'explore_maze.yaml')
    if not os.path.isfile(src):
        return
    path = ('/**', 'ros__parameters', 'progress_timeout')
    files = param_files.prepare_run_files(str(tmp_path), {'explore': src}, {'explore': {path: 33.0}})
    written = files['explore']
    assert written.startswith(str(tmp_path))
    data = param_files.load(written)
    assert param_files.get(data, path) == 33.0
    original = param_files.load(src)
    # everything else is unchanged
    assert param_files.apply(original, {path: 33.0}) == data
    assert 'progress_timeout = 33.0' in open(written).read()


def test_every_param_file_survives_a_round_trip(tmp_path):
    for key, (_, rel, _) in param_files.PARAM_FILES.items():
        src = os.path.join(REPO, 'maze_navigation', rel)
        if not os.path.isfile(src):
            continue
        data = param_files.load(src)
        leaves = param_files.leaves(data)
        assert leaves, key
        first_path, first_value = leaves[0]
        dest = str(tmp_path / f'{key}.yaml')
        param_files.write_with_overrides(src, {first_path: first_value}, dest)
        assert param_files.load(dest) == data


def test_keyboard_velocity():
    v = teleop.velocity
    assert v(set(), 0.2, 1.0) == (0.0, 0.0)
    assert v({teleop.FORWARD}, 0.2, 1.0) == (0.2, 0.0)
    assert v({teleop.BACK, teleop.LEFT}, 0.2, 1.0) == (-0.2, 1.0)
    assert v({teleop.FORWARD, teleop.BACK, teleop.RIGHT}, 0.2, 1.0) == (0.0, -1.0)
    lin, ang = v({teleop.FORWARD, teleop.LEFT}, 0.2, 1.0, slow=True)
    assert abs(lin - 0.2 * teleop.SLOW_FACTOR) < 1e-9 and abs(ang - teleop.SLOW_FACTOR) < 1e-9
