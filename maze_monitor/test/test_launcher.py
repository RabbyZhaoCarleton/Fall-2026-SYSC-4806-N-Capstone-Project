"""Starting and stopping a process group the way the Run panel does, with stand-in programs."""

import os
import sys
import textwrap
import time

from maze_monitor import launcher


def python(code):
    return [sys.executable, '-c', textwrap.dedent(code)]


def test_output_is_captured_and_logged(tmp_path):
    log = tmp_path / 'run' / 'launch.log'
    p = launcher.ManagedProcess('t')
    p.start(python('''
        import sys
        print("hello")
        print("\\x1b[31m[ERROR] colored\\x1b[0m")
        sys.exit(3)
    '''), log_path=str(log))
    assert p.wait(10)
    assert p.returncode == 3
    assert not p.user_stopped
    lines = p.drain()
    assert lines == ['hello', '[ERROR] colored']
    text = log.read_text()
    assert 'hello' in text and '[ERROR] colored' in text and text.startswith('$ ')


def test_stop_sends_sigint_to_the_whole_group(tmp_path):
    # A launch-like parent that starts a child and shuts it down on Ctrl+C.
    marker = tmp_path / 'clean'
    p = launcher.ManagedProcess('t', stop_timeout=10)
    p.start(python(f'''
        import signal, subprocess, sys, time
        child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
        def stop(*_):
            child.wait()
            open({str(marker)!r}, "w").write("ok")
            sys.exit(0)
        signal.signal(signal.SIGINT, stop)
        print("ready", flush=True)
        time.sleep(60)
    '''))
    deadline = time.monotonic() + 10
    while 'ready' not in p.tail and time.monotonic() < deadline:
        time.sleep(0.05)
    p.stop()
    assert p.state == launcher.STOPPING
    assert p.wait(10)
    assert p.user_stopped
    assert marker.read_text() == 'ok'  # the child got SIGINT too, so the parent's wait returned


def test_stop_escalates_when_sigint_is_ignored():
    p = launcher.ManagedProcess('t', stop_timeout=0.5, term_timeout=0.5)
    p.start(python('''
        import signal, time
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        print("ready", flush=True)
        time.sleep(60)
    '''))
    deadline = time.monotonic() + 10
    while 'ready' not in p.tail and time.monotonic() < deadline:
        time.sleep(0.05)
    t0 = time.monotonic()
    p.stop()
    assert p.wait(10)
    assert p.returncode == -9          # SIGKILL
    assert time.monotonic() - t0 >= 0.9


def test_leftovers_are_stopped_when_the_leader_exits_by_itself():
    # The leader exits, its child (like a Gazebo server after a crash) keeps running.
    p = launcher.ManagedProcess('t', stop_timeout=5)
    p.start(python('''
        import subprocess, sys
        subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
        print("started child", flush=True)
    '''))
    assert p.wait(15)
    assert p.returncode == 0
    assert not p.user_stopped
    try:
        os.killpg(p.pid, 0)
        raise AssertionError('the child is still running')
    except ProcessLookupError:
        pass


def test_stop_and_wait_and_restart():
    p = launcher.ManagedProcess('t', stop_timeout=5)
    for _ in range(2):
        p.start(python('import time; time.sleep(60)'))
        assert p.active
        assert p.stop_and_wait()
        assert p.state == launcher.EXITED and not p.active


def test_start_twice_is_refused():
    p = launcher.ManagedProcess('t')
    p.start(python('import time; time.sleep(60)'))
    try:
        p.start(python('pass'))
        raise AssertionError('started twice')
    except RuntimeError:
        pass
    finally:
        p.stop_and_wait()


def test_find_and_stop_leftovers():
    import subprocess
    tag = f'maze-monitor-leftover-test-{os.getpid()}'
    proc = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)', tag])
    try:
        found = launcher.find_leftovers(patterns=(tag,))
        assert [pid for pid, _ in found] == [proc.pid]
        assert launcher.stop_processes([proc.pid], timeout=5) == []
        proc.wait(5)
        assert launcher.find_leftovers(patterns=(tag,)) == []
    finally:
        if proc.poll() is None:
            proc.kill()


def test_leftover_patterns():
    import re
    pats = [re.compile(p) for p in launcher.LEFTOVER_PATTERNS]

    def hit(cmd):
        return any(p.search(cmd) for p in pats)

    assert hit('gz sim -r -s -v2 /home/u/ros2_ws/install/maze_sim/share/maze_sim/worlds/maze.sdf')
    assert hit('/usr/bin/gz sim -g -v2')
    assert hit('/opt/ros/jazzy/bin/ros2 launch maze_sim maze.launch.py explore:=true')
    assert not hit('ros2 run maze_monitor maze_monitor')
    assert not hit('code ~/ros2_ws/src/capstone/maze_sim')
