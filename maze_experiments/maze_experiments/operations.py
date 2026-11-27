"""Operations shared by the command line and the GUI (GR1, FNR2)."""

import os
import shutil
import signal
import time

from . import configuration, experiment as exp_mod, leases, manager_control, profiles, \
    provenance, resources


class Suite:
    """The data directory with its configurations and experiments."""

    def __init__(self, data_dir):
        self.data_dir = data_dir
        os.makedirs(data_dir, exist_ok=True)
        self.configs = configuration.ConfigStore(data_dir)
        self.experiments = exp_mod.ExperimentStore(data_dir)

    # ---------------------------------------------------------------- BR12 on start-up
    def find_orphans(self, cleanup=True):
        """Experiments that were running without a live manager. Their leftover run
        processes are stopped and they are marked interrupted (resumable)."""
        out = []
        for exp in self.experiments.list():
            if exp.state not in exp_mod.ACTIVE_STATES:
                continue
            if manager_control.manager_alive(exp):
                continue
            # Re-read: a manager may have just finished.
            try:
                exp.reload()
            except (OSError, ValueError):
                continue
            if exp.state not in exp_mod.ACTIVE_STATES or manager_control.manager_alive(exp):
                continue
            killed = stop_leftovers(exp) if cleanup else 0
            exp.log('orphan_found', previous_state=exp.state, leftover_processes=killed)
            exp.set_state('interrupted', reason='no live manager (manager, WSL or PC stopped)')
            out.append(exp)
        return out

    # ---------------------------------------------------------------- run
    def prepare_run(self, config, cpus=None, memory=None, disk=None, max_concurrency=None,
                    recalibrate=False):
        """Machine, allocation, calibration, estimates and every warning, before starting."""
        machine = resources.measure_machine(self.data_dir)
        alloc = resources.default_allocation(machine)
        if cpus:
            alloc['cpus'] = int(cpus)
        if memory:
            alloc['memory_bytes'] = resources.parse_size(memory)
        if disk:
            alloc['disk_bytes'] = resources.parse_size(disk)
        errors, warnings = configuration.check(config.data, self.data_dir, config.path)
        if machine.get('logical_cpus') and alloc['cpus'] > machine['logical_cpus']:
            errors.append(f'{alloc["cpus"]} CPUs requested; this machine has '
                          f'{machine["logical_cpus"]}.')
        if machine.get('memory_available') and alloc['memory_bytes'] > \
                machine['memory_available']:
            warnings.append(f'{resources.fmt_size(alloc["memory_bytes"])} of memory requested; '
                            f'{resources.fmt_size(machine["memory_available"])} is available.')
        renderer = machine.get('renderer')
        if renderer is None:
            warnings.append('The renderer could not be determined (tools/check_gpu.py failed).')
        elif renderer.get('kind') == 'software':
            warnings.append('Gazebo will render the lidar on the CPU (llvmpipe). Runs will be '
                            'slow; see Troubleshooting in README.md to use the GPU.')
        warnings += resources.clock_warnings()
        try:
            profile = profiles.load(config.data['robot_profile'], self.data_dir)
        except Exception:
            profile = None
        cal = None
        if profile and not recalibrate:
            cal = resources.load_calibration(self.data_dir, machine, profile.name)
        execution = {
            'cpus': alloc['cpus'], 'memory_bytes': alloc['memory_bytes'],
            'disk_bytes': alloc['disk_bytes'], 'max_concurrency': max_concurrency,
            'recalibrate': bool(recalibrate), 'machine': machine,
        }
        est = resources.estimate(config, cal, execution, machine)
        if est:
            if alloc['disk_bytes'] and est['disk_bytes'] > alloc['disk_bytes']:
                errors.append(f'The estimated disk use ({resources.fmt_size(est["disk_bytes"])}) '
                              f'exceeds the allocation ({resources.fmt_size(alloc["disk_bytes"])}). '
                              f'Allocate more, or use recording profile metrics_only or a '
                              f'smaller retention.')
            if machine.get('disk_free') and est['disk_bytes'] > machine['disk_free']:
                errors.append('The estimated disk use exceeds the free disk space.')
        return {'machine': machine, 'execution': execution, 'calibration': cal,
                'estimate': est, 'errors': errors, 'warnings': warnings, 'profile': profile}

    def start(self, config, prep, name=None, foreground=False, clock_jumps='retry'):
        profile = prep['profile']
        param_files = {f'profile.{k}': v for k, v in profile.stack_params().items()}
        for k, s in enumerate(config.sets()):
            for key, path in config.stack_param_files(s).items():
                param_files[f'set{k}.{key}'] = path
        prov = provenance.collect(profile, param_files, renderer=prep['machine'].get('renderer'))
        execution = dict(prep['execution'])
        execution['clock_jump_action'] = clock_jumps
        exp = self.experiments.create(config, execution, prov, name=name)
        exp.log('created', config=config.id, execution={k: v for k, v in execution.items()
                                                         if k != 'machine'},
                warnings=prep['warnings'])
        if foreground:
            return exp, None
        pid = manager_control.start_manager(exp, self.data_dir)
        return exp, pid

    def resume(self, exp):
        if manager_control.manager_alive(exp):
            if exp.state == 'paused':
                exp.request('resume')
                return 'resume requested'
            return 'already running'
        if exp.state in exp_mod.FINAL_STATES:
            raise ValueError(f'The experiment is {exp.state}; it cannot be resumed.')
        manager_control.start_manager(exp, self.data_dir, resume=True)
        return 'manager started'

    def pause(self, exp):
        if not manager_control.manager_alive(exp):
            raise ValueError('No manager is running this experiment.')
        exp.request('pause')

    def cancel(self, exp):
        if manager_control.manager_alive(exp):
            exp.request('cancel')
            return 'cancel requested'
        if exp.state not in exp_mod.FINAL_STATES:
            stop_leftovers(exp)
            exp.set_state('cancelled', reason='cancelled without a live manager')
        return 'cancelled'

    def delete_experiment(self, exp):
        self.experiments.delete(exp)

    def import_folder(self, folder):
        folder = os.path.abspath(os.path.expanduser(folder))
        if os.path.isfile(os.path.join(folder, exp_mod.EXPERIMENT_FILE)):
            exp, made_config = self.experiments.import_folder(folder, self.configs)
            return 'experiment', exp, made_config
        if os.path.isfile(os.path.join(folder, configuration.CONFIG_FILE)):
            cfg, new = self.configs.import_folder(folder)
            return 'configuration', cfg, new
        raise ValueError(f'{folder} is neither an experiment nor a configuration folder')

    def reprocess(self, exp, results_only=False, progress=None):
        from .metrics.results import build_results
        from .metrics.run_metrics import process_run
        if manager_control.manager_alive(exp):
            raise ValueError('The experiment is running; reprocess it when it has ended.')
        tol = (exp.config.data.get('analysis') or {}).get('exit_match_tolerance_m')
        errors = []
        if not results_only:
            rids = exp.run_ids()
            for i, rid in enumerate(rids):
                rec = exp_mod.read_run_record(exp.run_dir(rid))
                if not rec.get('outcome'):
                    continue
                if not os.path.isdir(os.path.join(exp.run_dir(rid), 'bag')) and \
                        rec.get('has_metrics'):
                    continue   # recording deleted by the retention policy: keep its metrics
                try:
                    process_run(exp.run_dir(rid), tolerance=tol)
                except Exception as exc:
                    errors.append(f'{rid}: {exc}')
                if progress:
                    progress(i + 1, len(rids))
        data = build_results(exp)
        from .metrics.aggregate import experiment_summary
        exp.reload()
        exp.meta['summary'] = experiment_summary(exp)
        exp.save()
        return data, errors

    def export(self, exp, out_dir, raw=None):
        from .metrics.results import all_json, build_results, load_results, markdown
        data = load_results(exp) or build_results(exp)
        os.makedirs(out_dir, exist_ok=True)
        base = os.path.join(out_dir, exp.id)
        with open(base + '.md', 'w') as f:
            f.write(markdown(exp, data, raw=raw))
        shutil.copyfile(os.path.join(exp.results_dir, 'runs.csv'), base + '.csv')
        with open(base + '.json', 'w') as f:
            f.write(all_json(exp, data))
        return [base + '.md', base + '.csv', base + '.json']

    def zip_experiment(self, exp, out_dir):
        os.makedirs(out_dir, exist_ok=True)
        return shutil.make_archive(os.path.join(out_dir, exp.id), 'zip',
                                   root_dir=os.path.dirname(exp.path),
                                   base_dir=os.path.basename(exp.path))


def stop_leftovers(exp):
    """Stop the run processes of an experiment whose manager is gone (BR12).

    A process is stopped only if its environment carries this experiment's run
    marker and it started after the run did, so a reused PID is never hit.
    """
    from .manager import Manager, STOP_SIGINT_WAIT
    total = 0
    for rid in exp.run_ids():
        run_dir = exp.run_dir(rid)
        rec = exp_mod.read_run_record(run_dir)
        if rec.get('outcome'):
            continue
        try:
            from . import fileio
            spec = fileio.read_json(os.path.join(run_dir, 'run_spec.json'))
        except (OSError, ValueError):
            continue
        if spec.get('boot_id') and spec['boot_id'] != resources.boot_id():
            continue
        marker = f'{leases.ENV_MARKER}={spec["experiment_id"]}/{spec["run_id"]}'.encode()
        start = spec.get('leader_start')
        pids = Manager.marked_processes(marker, min_start=start)
        total += len(pids)
        for sig, wait in ((signal.SIGINT, STOP_SIGINT_WAIT), (signal.SIGTERM, 5.0),
                          (signal.SIGKILL, 5.0)):
            pids = Manager.marked_processes(marker, min_start=start)
            if not pids:
                break
            for pid in pids:
                try:
                    os.kill(pid, sig)
                except OSError:
                    pass
            deadline = time.monotonic() + wait
            while time.monotonic() < deadline and Manager.marked_processes(marker,
                                                                             min_start=start):
                time.sleep(0.3)
        if not Manager.marked_processes(marker, min_start=start) and spec.get('domain'):
            leases.release(spec['domain'], f'{spec["experiment_id"]}/{spec["run_id"]}')
    return total
