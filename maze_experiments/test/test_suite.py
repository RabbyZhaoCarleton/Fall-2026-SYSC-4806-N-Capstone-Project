"""Modules B, C, D, F tests without a ROS graph or a simulation (S4)."""

import json
import os
import time

import pytest
import yaml

from maze_experiments import configuration, fileio, ids, paths, resources
from maze_experiments.experiment import Experiment, ExperimentStore, plan_runs, read_run_record
from maze_experiments.manager import Manager, Slot


def raw_config(**over):
    data = {'name': 'test', 'maze_sets': [
        {'name': 'a', 'maze': {'width': 2.0, 'length': 2.0, 'exits': 1}, 'mazes': 3,
         'repeats': 2, 'seed': 1},
        {'name': 'b', 'maze': {'width': 2.5, 'length': 2.5, 'exits': [1, 2]}, 'mazes': 2,
         'repeats': 1, 'seed': 2}]}
    data.update(over)
    return data


# ---------------------------------------------------------------- Module D: files
def test_atomic_json_and_schema(tmp_path):
    p = tmp_path / 'x.json'
    fileio.write_json(str(p), {'a': 1})
    assert fileio.read_json(str(p))['schema_version'] == fileio.SCHEMA_VERSION
    p.write_text(json.dumps({'schema_version': 99}))
    with pytest.raises(fileio.SchemaError, match='needs a newer'):
        fileio.read_json(str(p))
    p.write_text('{}')
    with pytest.raises(fileio.SchemaError, match='no schema_version'):
        fileio.read_json(str(p))
    assert not [f for f in os.listdir(tmp_path) if f.endswith('.tmp')]


def test_jsonl_is_append_only_and_survives_a_torn_line(tmp_path):
    p = str(tmp_path / 'log.jsonl')
    fileio.append_jsonl(p, {'n': 1})
    fileio.append_jsonl(p, {'n': 2})
    with open(p, 'a') as f:
        f.write('{"n": 3, "trunc')            # a crash in the middle of a write
    recs, offset = fileio.read_jsonl_from(p)
    assert [r['n'] for r in recs] == [1, 2]
    with open(p, 'a') as f:
        f.write('ated": 1}\n')
    more, _ = fileio.read_jsonl_from(p, offset)
    assert more == [{'n': 3, 'truncated': 1}]
    with open(p) as f:
        assert json.loads(f.readline())['schema_version'] == fileio.SCHEMA_VERSION


def test_ids_sort_by_time_and_differ():
    a, b = ids.new_id('x'), ids.new_id('x')
    assert a != b and a[:15] == b[:15]
    assert ids.parse_run_id(ids.run_id(1, 23, 4, 2)) == (1, 23, 4, 2)


def test_data_dir_refuses_windows_drive():
    with pytest.raises(paths.DataDirError):
        paths.check_data_dir('/mnt/c/Users/x/maze_experiments')


# ---------------------------------------------------------------- Module F: configurations
def test_sweeps_expand_every_combination():
    data = configuration.normalize(raw_config())
    data['maze_sets'][1]['maze']['corridor_width'] = [0.48, 0.98]
    sets = configuration.expand_sets(data)
    assert len(sets) == 1 + 2 * 2
    assert sets[1]['name'] == 'b (corridor_width=0.48, exits=1)'
    assert configuration.total_runs(data) == 6 + 4 * 2


def test_check_reports_every_problem_at_once(tmp_path):
    data = configuration.normalize(raw_config())
    data['maze_sets'][0]['maze']['corridor_width'] = 0.45      # pitch doesn't divide
    data['maze_sets'][1]['maze']['exits'] = 40                 # too many exits
    data['run_limits']['sim_timeout_s'] = -1
    data['recording']['profile'] = 'everything'
    errors, _ = configuration.check(data, str(tmp_path), workspace=False)
    assert len(errors) == 4
    assert any('Nearest valid corridor widths' in e for e in errors)
    assert any('sim_timeout_s' in e for e in errors)
    assert any('recording.profile' in e for e in errors)


def test_configurations_are_immutable_and_guarded(tmp_path):
    store = configuration.ConfigStore(str(tmp_path))
    params = tmp_path / 'explore.yaml'
    params.write_text('explore_node:\n  ros__parameters: {}\n')
    raw = raw_config()
    raw['maze_sets'][0]['stack_params'] = {'explore_params': str(params)}
    cfg, _ = store.create(raw, check_workspace=False)
    # FR8: the parameter file is copied into the configuration
    copied = cfg.stack_param_files(cfg.data['maze_sets'][0])['explore_params']
    assert os.path.isfile(copied) and copied.startswith(cfg.path)
    edit = store.copy_for_edit(cfg)
    assert edit['name'] == 'test (modified)' and 'id' not in edit
    cfg2, _ = store.create(edit, derived_from=cfg.id, check_workspace=False)
    assert cfg2.id != cfg.id and cfg2.data['derived_from'] == cfg.id
    assert store.get(cfg.id).data == cfg.data           # the original is untouched

    class FakeExp:
        config_id = cfg.id
    with pytest.raises(configuration.ConfigError, match='Delete them first'):
        store.delete(cfg, [FakeExp()])
    store.delete(cfg2, [FakeExp()])
    assert len(store.list()) == 1


# ---------------------------------------------------------------- Module B: run order
def test_run_order_is_round_robin_and_deterministic():
    sets = [{'mazes': 3, 'repeats': 2}, {'mazes': 2, 'repeats': 1}]
    order = [p.key for p in plan_runs(sets)]
    assert order[:5] == [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0), (0, 2, 0)]
    assert order[5:] == [(0, 0, 1), (0, 1, 1), (0, 2, 1)]
    assert order == [p.key for p in plan_runs(sets)]          # FNR1


# ---------------------------------------------------------------- Module C
def test_concurrency_cap_is_the_minimum_of_the_limits():
    cal = {'avg_cpu_cores': 2.0, 'peak_memory_bytes': 2 * 2 ** 30}
    ex = {'cpus': 9, 'memory_bytes': 8 * 2 ** 30, 'max_concurrency': None}
    assert resources.concurrency_cap(ex, cal, 101) == 3         # memory: 8 / 2.5
    assert resources.concurrency_cap({**ex, 'memory_bytes': 64 * 2 ** 30}, cal, 101) == 4  # cpu
    assert resources.concurrency_cap({**ex, 'max_concurrency': 2}, cal, 101) == 2
    assert resources.concurrency_cap(ex, cal, 1) == 1
    assert resources.concurrency_cap(ex, None, 101) >= 1


def test_controller_raises_one_at_a_time_and_lowers_on_low_rtf():
    c = resources.ConcurrencyController(0.8, cap=3)
    t = 1000.0
    c.last_change = t - 100
    assert c.update([('a', t - 100, 0.95)], t)[0]['change'] == 'raise'
    assert c.target == 2
    # the new run must hold the minimum RTF for a while before the next raise
    assert c.update([('a', t - 100, 0.95), ('b', t - 5, 0.95)], t + 10) == []
    assert c.target == 2
    # a run below the minimum for 30 s lowers the target by one and holds back new runs
    c.update([('a', t, 0.95), ('b', t, 0.6)], t + 100)
    assert c.held_back and not c.may_start(1)
    changes = c.update([('a', t, 0.95), ('b', t, 0.6)], t + 131)
    assert changes and changes[0]['change'] == 'lower' and c.target == 1


def test_cpu_shares_are_disjoint():
    cpus = list(range(6, 12))
    shares = [resources.assign_cpus(cpus, i, 3) for i in range(3)]
    assert shares == [[6, 7], [8, 9], [10, 11]]


def test_size_parsing():
    assert resources.parse_size('8G') == 8 * 2 ** 30
    assert resources.parse_size('512MiB') == 512 * 2 ** 20
    assert resources.parse_size(1000) == 1000


# ---------------------------------------------------------------- Module B: outcomes
class FakeProc:
    def __init__(self, rc=None):
        self.rc = rc
        self.pid = 1

    def poll(self):
        return self.rc


def make_manager(tmp_path, **limits):
    store = ExperimentStore(str(tmp_path))
    cstore = configuration.ConfigStore(str(tmp_path))
    cfg, _ = cstore.create(raw_config(run_limits=limits), check_workspace=False)
    exp = store.create(cfg, {'cpus': 2})
    m = Manager(exp, str(tmp_path))
    m.min_rtf = 0.8
    return m


def slot_with(m, statuses=(), last=None, died=(), ready=True, final=None, rc=None, age=10.0):
    s = Slot(m.exp.plan()[0], 1)
    s.proc = FakeProc(rc)
    s.started = time.monotonic() - age
    s.statuses = list(statuses)
    s.died = list(died)
    s.last = last or {'sim': 50.0, 'area': 1.0, 'pos': [0.0, 0.0], 'launch': 'running',
                      'ready': {k: ready for k in ('clock', 'scan', 'ground_truth',
                                                   'explore_started')}}
    s.final = final
    return s


def test_outcomes_from_explore_status(tmp_path):
    m = make_manager(tmp_path)
    done = {'final': True}
    cases = [
        (['exploration_started', 'exploration_complete', 'returning_to_origin',
          'returned_to_origin'], 'complete'),
        (['exploration_started', 'exploration_incomplete', 'returning_to_origin',
          'return_failed'], 'incomplete'),
        (['exploration_started', 'exploration_complete', 'return_failed'], 'return_failed'),
        (['exploration_started'], 'stack_crash'),
    ]
    for statuses, expected in cases:
        s = slot_with(m, statuses, final=done, rc=0)
        s.ready_at = time.monotonic()
        assert m.outcome_from_statuses(s, False)[0] == expected
    s = slot_with(m, [], final=done, rc=0, ready=False)
    assert m.outcome_from_statuses(s, False)[0] == 'startup_failed'
    s = slot_with(m, ['exploration_started'], final=None, rc=-9)
    assert m.outcome_from_statuses(s, True)[0] == 'infra_failure'


def test_limits(tmp_path):
    m = make_manager(tmp_path, sim_timeout_s=60, startup_timeout_s=120, stall_window_s=120)
    now = time.monotonic()
    # BR3: not ready within the startup timeout
    s = slot_with(m, ready=False, age=121)
    assert m.evaluate(s, now)[0] == 'startup_failed'
    # BR5: sim-time timeout
    s = slot_with(m, last={'sim': 61.0, 'area': 1.0, 'pos': [0, 0], 'launch': 'running',
                           'ready': {'clock': True, 'scan': True, 'ground_truth': True,
                                     'explore_started': True}})
    assert m.evaluate(s, now)[0] == 'timeout'
    # BR6: wall limit = startup + sim timeout / min RTF = 120 + 75 s
    s = slot_with(m, age=196, last={'sim': 10.0, 'area': 1.0, 'pos': [0, 0],
                                    'launch': 'running', 'ready': {'clock': True, 'scan': True,
                                                                   'ground_truth': True,
                                                                   'explore_started': True}})
    assert m.evaluate(s, now)[0] == 'sim_frozen'
    # a stack process dying while the run is active
    s = slot_with(m, ['exploration_started'],
                  died=[{'process': 'controller_server-9', 'exit_code': -11,
                         'after_stop': False}])
    assert m.evaluate(s, now)[0] == 'stack_crash'
    # ... but not while the launch shuts down after explore_lite ended
    s = slot_with(m, ['exploration_started'],
                  died=[{'process': 'gazebo-2', 'exit_code': -15, 'after_stop': True}])
    assert m.evaluate(s, now) is None


def test_stall_detection(tmp_path):
    m = make_manager(tmp_path, sim_timeout_s=900, stall_window_s=120)
    now = time.monotonic()
    ready = {'clock': True, 'scan': True, 'ground_truth': True, 'explore_started': True}
    s = slot_with(m)
    assert m.evaluate(s, now) is None             # becomes ready, sets the reference
    for sim, area, pos, expect in ((60, 1.02, [0.1, 0], None),     # tiny changes only
                                   (100, 1.5, [0.1, 0], None),     # map grew: new reference
                                   (200, 1.52, [0.2, 0.0], None),  # 100 s since progress
                                   (221, 1.53, [0.2, 0.1], 'stalled')):
        s.last = {'sim': sim, 'area': area, 'pos': pos, 'launch': 'running', 'ready': ready}
        res = m.evaluate(s, now)
        assert (res[0] if res else None) == expect, sim


def test_queue_skips_robot_results_and_counts_retries(tmp_path):
    m = make_manager(tmp_path)
    exp = m.exp
    plan = exp.plan()

    def record(planned, attempt, outcome, reason=''):
        d = exp.run_dir(planned.run_id(attempt))
        os.makedirs(d)
        with open(os.path.join(d, 'run.json'), 'w') as f:
            json.dump({'experiment': {'key': list(planned.key), 'attempt': attempt,
                                      'outcome': outcome, 'reason': reason}}, f)
    record(plan[0], 1, 'complete')
    record(plan[1], 1, 'startup_failed')
    record(plan[2], 1, 'sim_frozen')
    record(plan[2], 2, 'infra_failure')
    record(plan[2], 3, 'startup_failed')       # 3 non-robot attempts > 2 retries: given up
    record(plan[3], 1, 'interrupted', 'manager_crashed')   # doesn't use a retry
    m.retries = 2
    m.build_queue()
    queued = {p.key: a for p, a in m.queue}
    assert plan[0].key not in queued and plan[2].key not in queued
    assert queued[plan[1].key] == 2 and queued[plan[3].key] == 2
    assert len(m.queue) == len(plan) - 2
    assert [p.order for p, _ in m.queue] == sorted(p.order for p, _ in m.queue)


def test_recover_marks_active_attempts_interrupted(tmp_path):
    m = make_manager(tmp_path)
    exp = m.exp
    d = exp.run_dir(exp.plan()[0].run_id(1))
    os.makedirs(d)
    with open(os.path.join(d, 'run.json'), 'w') as f:
        json.dump({'format': 1, 'status': 'recording', 'experiment': {'attempt': 1}}, f)
    m.recover()
    rec = read_run_record(d)
    assert rec['outcome'] == 'interrupted' and rec['reason'] == 'manager_crashed'
    m.retries = 2
    m.build_queue()
    assert (exp.plan()[0], 2) in [(p, a) for p, a in m.queue if p.key == exp.plan()[0].key] \
        or any(p.key == exp.plan()[0].key and a == 2 for p, a in m.queue)


# ---------------------------------------------------------------- DR4: sharing
def test_import_recreates_a_missing_configuration(tmp_path):
    a, b = tmp_path / 'pc1', tmp_path / 'pc2'
    cstore = configuration.ConfigStore(str(a))
    cfg, _ = cstore.create(raw_config(), check_workspace=False)
    exp = ExperimentStore(str(a)).create(cfg, {'cpus': 1})
    exp.log('created')
    import shutil
    copy = tmp_path / 'usb' / exp.id
    shutil.copytree(exp.path, copy)
    store_b, cstore_b = ExperimentStore(str(b)), configuration.ConfigStore(str(b))
    imported, made = store_b.import_folder(str(copy), cstore_b)
    assert made and imported.id == exp.id
    assert cstore_b.get(cfg.id).data['name'] == 'test'
    assert len(imported.plan()) == len(exp.plan())
    # Inside the folder only relative paths are used
    with open(os.path.join(imported.path, 'experiment.json')) as f:
        assert str(a) not in f.read()


def test_listing_1000_experiments_is_fast(tmp_path):
    root = tmp_path / 'experiments'
    for i in range(1000):
        d = root / f'e{i:04d}'
        d.mkdir(parents=True)
        (d / 'experiment.json').write_text(json.dumps({
            'schema_version': 1, 'id': d.name, 'created': f'2026-01-01T00:{i // 60:02d}:'
            f'{i % 60:02d}', 'state': 'completed', 'summary': {}}))
    t = time.perf_counter()
    exps = ExperimentStore(str(tmp_path)).list()
    assert len(exps) == 1000 and time.perf_counter() - t < 2.0   # DNR2


def test_controller_stops_probing_a_level_that_failed_twice():
    c = resources.ConcurrencyController(0.8, cap=3)
    t = 0.0
    for _ in range(2):
        c.target = 2
        c.last_change = t - 100
        c.update([('a', t - 100, 0.95), ('b', t - 100, 0.5)], t)
        t += 31
        c.update([('a', t - 100, 0.95), ('b', t - 100, 0.5)], t)
        assert c.target == 1
        t += 10000
    assert c.cap == 1
    c.last_change = t - 100
    assert c.update([('a', t - 100, 0.95)], t) == [] and c.target == 1

