"""The run dialog (2.3.1, CR1, CR2, CR9): what this machine has, how much to allocate."""

from python_qt_binding.QtWidgets import (QCheckBox, QDialog, QDialogButtonBox, QDoubleSpinBox,
                                         QFormLayout, QLabel, QSpinBox, QVBoxLayout)

from .. import resources
from ..metrics import fmt
from .common import background, heading, style

GB = 2 ** 30


def machine_text(m):
    r = m.get('renderer') or {}
    g = m.get('gpu_memory_mb')
    lines = [
        f'CPU: {m.get("cpu_model") or "unknown"}',
        f'Logical CPUs (available to WSL): {m.get("logical_cpus") or "unknown"}',
        f'Memory available to WSL: {fmt.size(m.get("memory_available"))} of '
        f'{fmt.size(m.get("memory_total"))}',
        f'Free disk in the data directory: {fmt.size(m.get("disk_free"))}',
        f'Renderer: {r.get("name", "unknown")}' + (
            ' — software rendering on the CPU, runs will be slow' if r.get('kind') == 'software'
            else ''),
        f'GPU memory: {f"{g["total"]} MB ({g["used"]} MB in use)" if g else "unknown"}',
    ]
    return '\n'.join(lines)


class RunDialog(QDialog):
    def __init__(self, win, config):
        super().__init__(win)
        self.win = win
        self.config = config
        self.prep = None
        self.experiment = None
        self.setWindowTitle(f'Run "{config.name}"')
        self.setMinimumSize(640, 640)
        lay = QVBoxLayout(self)
        lay.addWidget(heading(f'Run "{config.name}"', 2))
        lay.addWidget(QLabel(f'{config.total_runs()} runs in {len(config.sets())} maze set(s).'))
        self.machine_label = QLabel('Measuring this machine…')
        self.machine_label.setWordWrap(True)
        lay.addWidget(heading('This machine', 1))
        lay.addWidget(self.machine_label)
        lay.addWidget(heading('Allocation', 1))
        form = QFormLayout()
        self.cpus = QSpinBox()
        self.memory = QDoubleSpinBox()
        self.memory.setSuffix(' GB')
        self.memory.setDecimals(1)
        self.disk = QDoubleSpinBox()
        self.disk.setSuffix(' GB')
        self.disk.setDecimals(0)
        self.disk.setMaximum(1e6)
        self.max_conc = QSpinBox()
        self.max_conc.setRange(1, 101)
        self.max_conc.setValue(101)
        self.max_conc.setToolTip('Never run more than this many simulations at once')
        self.recal = QCheckBox('Measure a new calibration (ignore the saved one)')
        self.flag_jumps = QCheckBox('Only flag runs during WSL clock jumps (do not repeat them)')
        self.flag_jumps.setToolTip('The specification repeats such runs (BR13). Flagging keeps '
                                   'them, for machines whose clock cannot be fixed.')
        for w in (self.cpus, self.memory, self.disk, self.max_conc):
            w.setMinimumHeight(26)
        form.addRow('CPUs', self.cpus)
        form.addRow('Memory', self.memory)
        form.addRow('Disk space', self.disk)
        form.addRow('At most at once', self.max_conc)
        form.addRow('', self.recal)
        form.addRow('', self.flag_jumps)
        lay.addLayout(form)
        self.estimate = QLabel('')
        self.estimate.setWordWrap(True)
        lay.addWidget(self.estimate)
        self.problems = QLabel('')
        self.problems.setWordWrap(True)
        lay.addWidget(self.problems)
        self.buttons = QDialogButtonBox(QDialogButtonBox.Cancel)
        self.start_btn = self.buttons.addButton('Start', QDialogButtonBox.AcceptRole)
        self.start_btn.setEnabled(False)
        self.buttons.accepted.connect(self.start)
        self.buttons.rejected.connect(self.reject)
        lay.addWidget(self.buttons)
        for w in (self.cpus, self.memory, self.disk, self.max_conc):
            w.valueChanged.connect(self.update_estimate)
        self.recal.toggled.connect(self.update_estimate)
        background(lambda: win.suite.prepare_run(config), self.got_prep, parent=self)

    def got_prep(self, prep):
        self.prep = prep
        m = prep['machine']
        self.machine_label.setText(machine_text(m))
        ex = prep['execution']
        self.cpus.setRange(1, m.get('logical_cpus') or 64)
        self.cpus.setValue(ex['cpus'])
        self.memory.setRange(0.5, max((m.get('memory_total') or 0) / GB, 1.0))
        self.memory.setValue(ex['memory_bytes'] / GB)
        self.disk.setRange(1, max((m.get('disk_free') or 0) / GB, 1.0))
        self.disk.setValue(ex['disk_bytes'] / GB)
        self.start_btn.setEnabled(True)
        self.update_estimate()

    def execution(self):
        ex = dict(self.prep['execution'])
        ex.update(cpus=self.cpus.value(), memory_bytes=int(self.memory.value() * GB),
                  disk_bytes=int(self.disk.value() * GB),
                  max_concurrency=self.max_conc.value() if self.max_conc.value() < 101 else None,
                  recalibrate=self.recal.isChecked())
        return ex

    def update_estimate(self):
        if self.prep is None:
            return
        ex = self.execution()
        cal = None if self.recal.isChecked() else self.prep['calibration']
        est = resources.estimate(self.config, cal, ex, self.prep['machine'])
        if est:
            self.estimate.setText(
                f'Estimate: about {fmt.long_duration(est["duration_s"])} with up to '
                f'{est["concurrency"]} runs at once, and {fmt.size(est["disk_bytes"])} of disk. '
                f'Concurrency starts at 1 and rises only while every run keeps the minimum '
                f'real-time factor.')
        else:
            self.estimate.setText('Estimate after the first run: no calibration for this machine '
                                  'yet. The first run runs alone and measures what a run costs '
                                  'here.')
        lines = [f'<span style="color:{style.RED.name()}">Error: {e}</span>'
                 for e in self.prep['errors']]
        lines += [f'<span style="color:{style.AMBER.name()}">Warning: {w}</span>'
                  for w in self.prep['warnings']]
        self.problems.setText('<br>'.join(lines))
        self.start_btn.setEnabled(not self.prep['errors'])

    def start(self):
        prep = dict(self.prep)
        prep['execution'] = self.execution()
        self.start_btn.setEnabled(False)

        def done(res):
            self.experiment = res[0]
            self.accept()

        def failed(text):
            self.start_btn.setEnabled(True)
            self.problems.setText(f'<span style="color:{style.RED.name()}">Could not start: '
                                  f'{text.splitlines()[0]}</span>')
        background(lambda: self.win.suite.start(
            self.config, prep, clock_jumps='flag' if self.flag_jumps.isChecked() else 'retry'),
            done, failed, parent=self, progress_text='Starting the experiment…')
