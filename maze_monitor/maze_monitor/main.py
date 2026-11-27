"""Entry point: ros2 run maze_monitor maze_monitor [options]."""

import argparse
import json
import signal
import sys


def parse_args(argv):
    p = argparse.ArgumentParser(
        prog='maze_monitor',
        description='Run manager and debugging monitor for the maze robot. Starts, stops, and '
                    'restarts simulation runs with the settings you choose, records each run, '
                    'shows live data, and replays saved runs.')
    p.add_argument('--replay', metavar='RUN',
                   help='open a saved run: a run folder, a folder name in the runs directory, '
                        'or "latest"')
    p.add_argument('--no-ros', action='store_true',
                   help='do not connect to ROS 2 (replay only)')
    p.add_argument('--start', action='store_true',
                   help='start a simulation run at once, with the last used settings '
                        '(or the --preset)')
    p.add_argument('--preset', metavar='NAME',
                   help='fill in the Run panel from this saved preset')
    p.add_argument('--record', action='store_true',
                   help='record the live data at once, for a system started some other way '
                        '(the real robot, or a launch in a terminal)')
    p.add_argument('--no-record', action='store_true',
                   help='with --attach: show live data without saving a run')
    p.add_argument('--attach', action='store_true',
                   help='set by maze.launch.py: record the run that launch started; the Run '
                        'panel then cannot start or stop runs')
    p.add_argument('--label', default='',
                   help='short label added to the run folder name, for example "explore"')
    p.add_argument('--launch-args', default='{}',
                   help='launch arguments as JSON, saved with the run (set by the launch file)')
    p.add_argument('--runs-dir', help='folder for saved runs (default: ~/maze_runs)')
    p.add_argument('--config', help='YAML file with settings that override the defaults')
    return p.parse_args(argv)


def main(argv=None):
    from rclpy.utilities import remove_ros_args
    argv = sys.argv if argv is None else argv
    args = parse_args(remove_ros_args(argv)[1:])

    from .config import Config
    config = Config.load(args.config, args.runs_dir)
    try:
        launch_args = json.loads(args.launch_args)
    except ValueError:
        launch_args = {'raw': args.launch_args}

    from python_qt_binding.QtCore import Qt, QTimer
    from python_qt_binding.QtWidgets import QApplication
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QApplication(argv[:1])
    app.setApplicationName('Maze monitor')
    app.setStyle('Fusion')
    from .gui import style
    app.setPalette(style.light_palette())

    bridge = None
    live = not args.no_ros
    if live:
        from .ros_bridge import RosBridge
        bridge = RosBridge(config, ros_args=argv)
        try:
            bridge.start()
        except Exception as exc:
            print(f'maze_monitor: could not start ROS 2 ({exc}); continuing in replay-only mode',
                  file=sys.stderr)
            bridge = None

    from .gui.main_window import MainWindow, Options
    record = (args.attach or args.record) and not args.no_record
    window = MainWindow(config, Options(live=bridge is not None, record=record,
                                        label=args.label, launch_args=launch_args,
                                        replay=args.replay, attached=args.attach,
                                        start=args.start, preset=args.preset), bridge)
    window.show()

    # Ctrl+C in the terminal or a launch shutdown: close the window cleanly so the
    # recording is finalized and a simulation started from the monitor is stopped
    # (it runs in its own process group, so the Ctrl+C does not reach it). Qt only
    # lets Python handle signals while Python code runs; the window's 50 ms timer
    # provides that.
    def on_signal(signum, _frame):
        window.quit_without_asking = True
        QTimer.singleShot(0, window.close)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    rc = app.exec_()
    window.shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
