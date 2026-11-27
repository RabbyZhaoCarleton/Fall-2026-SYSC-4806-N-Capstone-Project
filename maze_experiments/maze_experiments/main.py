"""maze_experiments: GUI with no arguments, command line with a subcommand (GR1, 2.3.5).

    maze_experiments                                 the graphical application
    maze_experiments config create my_config.yaml    save a configuration
    maze_experiments run my_config --cpus 6 --memory 8G
    maze_experiments status
    maze_experiments report <experiment> --format md

The GUI and the command line use the same library and the same files.
"""

import argparse
import sys


def build_parser():
    p = argparse.ArgumentParser(
        prog='maze_experiments',
        description='Experiment suite for the maze robot: generate mazes, run the simulated '
                    'robot on them many times, and summarize the results. Without a command, '
                    'opens the graphical application.')
    p.add_argument('--data-dir', help='data directory (default: ~/maze_experiments, or '
                                      '$MAZE_EXPERIMENTS_DIR)')
    sub = p.add_subparsers(dest='command', metavar='command')

    c = sub.add_parser('config', help='create, list, show, check or delete configurations')
    csub = c.add_subparsers(dest='config_command', metavar='action', required=True)
    x = csub.add_parser('create', help='save a configuration from a YAML file')
    x.add_argument('file')
    x.add_argument('--name', help='override the name in the file')
    x = csub.add_parser('check', help='check a YAML file without saving it (FR4)')
    x.add_argument('file')
    csub.add_parser('list', help='list configurations')
    x = csub.add_parser('show', help='print a configuration')
    x.add_argument('config')
    x = csub.add_parser('delete', help='delete a configuration without experiments')
    x.add_argument('config')
    x = csub.add_parser('example', help='print an example configuration file')

    r = sub.add_parser('run', help='run a configuration (starts a background manager)')
    r.add_argument('config', help='configuration id, name, or folder')
    r.add_argument('--cpus', type=int, help='logical CPUs to allocate (default: half)')
    r.add_argument('--memory', help='memory to allocate, e.g. 8G (default: half of available)')
    r.add_argument('--disk', help='disk space to allocate, e.g. 50G (default: half of free)')
    r.add_argument('--max-concurrency', type=int, help='never run more than this many at once')
    r.add_argument('--name', help='experiment name (default: the configuration name)')
    r.add_argument('--recalibrate', action='store_true',
                   help='measure a new calibration even if a recent one exists')
    r.add_argument('--clock-jumps', choices=('retry', 'flag'), default='retry',
                   help='runs during a WSL clock jump: repeat them (default, BR13) or only flag '
                        'them')
    r.add_argument('--foreground', action='store_true',
                   help='run the manager in this terminal instead of the background')
    r.add_argument('--yes', '-y', action='store_true', help='do not ask for confirmation')
    r.add_argument('--force', action='store_true',
                   help='start despite the errors the checks report (for example estimates '
                        'that do not fit the allocation)')

    s = sub.add_parser('status', help='show experiments, or one experiment in detail')
    s.add_argument('experiment', nargs='?')
    s.add_argument('--all', action='store_true', help='list every experiment')
    for name, helptext in (('pause', 'start no new runs; active runs finish'),
                           ('resume', 'resume a paused or interrupted experiment'),
                           ('cancel', 'stop active runs and end the experiment')):
        x = sub.add_parser(name, help=helptext)
        x.add_argument('experiment')

    x = sub.add_parser('report', help='print or save the results (md, csv, json, text)')
    x.add_argument('experiment')
    x.add_argument('--format', choices=('md', 'csv', 'json', 'text'), default='text')
    x.add_argument('--output', '-o', help='file to write (default: print)')
    x.add_argument('--raw-run', help='add the raw messages of this run (md only)')
    x.add_argument('--raw-topics', help='comma-separated topics for --raw-run')
    x.add_argument('--raw-max', type=int, default=20, help='messages per topic (default 20)')

    x = sub.add_parser('reprocess', help='recompute every run\'s metrics and the results (ER5)')
    x.add_argument('experiment')
    x.add_argument('--results-only', action='store_true',
                   help='only rebuild the aggregate results from the existing run metrics')

    x = sub.add_parser('import', help='import an experiment or configuration folder')
    x.add_argument('folder')

    x = sub.add_parser('export', help='write the Markdown, CSV and JSON results to a folder, '
                                      'or zip the experiment folder')
    x.add_argument('experiment')
    x.add_argument('--output', '-o', help='folder (default: the current folder)')
    x.add_argument('--zip', action='store_true', help='zip the whole experiment folder')

    x = sub.add_parser('delete', help='delete an experiment (asks first)')
    x.add_argument('experiment')
    x.add_argument('--yes', '-y', action='store_true')

    x = sub.add_parser('replay', help='open a run in maze_monitor --replay')
    x.add_argument('experiment')
    x.add_argument('run')

    x = sub.add_parser('machine', help='show what this machine has (Module C)')

    # Internal commands, started by the suite itself.
    x = sub.add_parser('_manager')
    x.add_argument('experiment')
    x.add_argument('--resume', action='store_true')
    x = sub.add_parser('_agent')
    x.add_argument('run_dir')
    x = sub.add_parser('_process-run')
    x.add_argument('run_dir')
    return p


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    # ros2 run adds --ros-args ...; drop them.
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = build_parser().parse_args(argv)
    from . import paths
    try:
        data_dir = paths.data_dir(args.data_dir)
    except paths.DataDirError as exc:
        print(f'maze_experiments: {exc}', file=sys.stderr)
        return 2
    if args.command is None:
        from .gui.app import main as gui_main
        return gui_main(data_dir)
    if args.command == '_agent':
        from . import agent
        return agent.main(args.run_dir)
    if args.command == '_manager':
        from . import manager
        return manager.main(args.experiment, data_dir, args.resume)
    if args.command == '_process-run':
        return process_run_command(args.run_dir)
    from . import cli
    return cli.dispatch(args, data_dir)


def process_run_command(run_dir):
    import os
    from . import fileio
    from .metrics.run_metrics import process_run
    exp_dir = os.path.dirname(os.path.dirname(os.path.abspath(run_dir)))
    tolerance = None
    try:
        from . import configuration
        cfg = fileio.read_yaml(os.path.join(exp_dir, configuration.CONFIG_FILE))
        tolerance = (cfg.get('analysis') or {}).get('exit_match_tolerance_m')
    except Exception:
        pass
    m = process_run(run_dir, tolerance=tolerance)
    print(f'{os.path.basename(run_dir)}: processed in {m.get("processing_s")} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
