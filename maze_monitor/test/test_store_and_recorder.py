import os
import subprocess
import sys
import textwrap

from rclpy.serialization import serialize_message
from std_msgs.msg import String

from maze_monitor.config import Config
from maze_monitor.recorder import Recorder, read_bag
from maze_monitor.run_store import RunStore
from maze_monitor.session import load_run


def test_create_list_rename_delete(tmp_path):
    store = RunStore(str(tmp_path))
    a = store.create('explore')
    b = store.create('explore')
    assert a.path != b.path
    assert {r.path for r in store.list()} == {a.path, b.path}
    store.rename(a, 'wall hit at the dead end')
    assert store.load(a.path).name == 'wall hit at the dead end'
    assert store.resolve(os.path.basename(b.path)).path == b.path
    store.delete(a)
    assert [r.path for r in store.list()] == [b.path]


def test_delete_refuses_outside_runs_dir(tmp_path):
    store = RunStore(str(tmp_path / 'runs'))
    other = RunStore(str(tmp_path / 'other')).create('x')
    try:
        store.delete(other)
    except ValueError:
        return
    raise AssertionError('deleted a folder outside the runs directory')


def test_record_and_replay_roundtrip(tmp_path):
    store = RunStore(str(tmp_path))
    run = store.create('test')
    run.update(start_ns=1_000_000_000)
    rec = Recorder(run.bag_path)
    for i in range(100):
        rec.write('/chatter', 'std_msgs/msg/String', serialize_message(String(data=str(i))),
                  1_000_000_000 + i * 10_000_000)
    rec.close()
    msgs = list(read_bag(run.bag_path))
    assert len(msgs) == 100
    session = load_run(Config.load(), store.load(run.path))
    tr = session.timeline.tracks['/chatter']
    assert len(tr) == 100
    assert abs(tr.times[-1] - 0.99) < 1e-6
    assert tr.latest(0.5)[1].data == '50'


def test_recording_survives_a_killed_process(tmp_path):
    bag = str(tmp_path / 'bag')
    code = textwrap.dedent(f'''
        import os, sys
        sys.path.insert(0, {os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))!r})
        from rclpy.serialization import serialize_message
        from std_msgs.msg import String
        from maze_monitor.recorder import Recorder
        rec = Recorder({bag!r})
        for i in range(500):
            rec.write('/chatter', 'std_msgs/msg/String', serialize_message(String(data='x')), 10 + i)
        os._exit(0)  # no close(): simulates a crash
    ''')
    subprocess.run([sys.executable, '-c', code], check=True)
    assert len(list(read_bag(bag))) == 500
