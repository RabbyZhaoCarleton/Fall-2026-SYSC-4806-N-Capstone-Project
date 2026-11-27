"""Charts for the results screen and the export (ER10), drawn with matplotlib (Agg).

Colors follow maze_monitor's style.py: green OK / complete, amber warning,
red error, blue intent, black reality, gray no data. Maze sets are not
colored (color carries meaning only); they are separate rows or labels.
"""

import os

GREEN, AMBER, RED, BLUE, BLACK, GRAY, PURPLE = ('#2e7d32', '#b26a00', '#c62828', '#1565c0',
                                                '#212121', '#8a8a8a', '#6a1b9a')
OUTCOME_COLORS = {'complete': GREEN, 'incomplete': AMBER, 'return_failed': '#e0a030',
                  'timeout': GRAY, 'stalled': '#b0b0b0', 'stack_crash': RED}
BOX_METRICS = [('time.mission_s', 'Mission time (min, sim)', 1 / 60.0),
               ('exploration.coverage', 'Coverage (%)', 100.0),
               ('exploration.distance_m', 'Distance driven (m)', 1.0),
               ('localization.slam.max_pos_m', 'SLAM max position error (cm)', 100.0),
               ('safety.min_clearance_m', 'Minimum clearance (cm)', 100.0)]


def _plt():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False,
                         'axes.edgecolor': '#999999', 'axes.titleweight': 'bold',
                         'figure.dpi': 100})
    return plt


def outcomes_per_set(agg, path):
    plt = _plt()
    from ..experiment import ROBOT_OUTCOMES
    sets = agg['sets']
    fig, ax = plt.subplots(figsize=(7.5, 0.5 + 0.45 * max(len(sets), 1)))
    names = [s['name'] for s in sets]
    left = [0] * len(sets)
    for o in ROBOT_OUTCOMES:
        vals = [s['outcomes'][o]['k'] for s in sets]
        if not any(vals):
            continue
        ax.barh(names, vals, left=left, color=OUTCOME_COLORS[o], label=o, height=0.6)
        left = [a + b for a, b in zip(left, vals)]
    ax.set_xlabel('runs')
    ax.invert_yaxis()
    ax.legend(loc='lower center', bbox_to_anchor=(0.5, 1.0), ncol=6, frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def distributions(agg, path):
    plt = _plt()
    from .aggregate import get_path
    counted = [r for r in agg['counted'] if r['metrics']]
    n_sets = len(agg['sets'])
    fig, axes = plt.subplots(1, len(BOX_METRICS), figsize=(11, 0.9 + 0.45 * max(n_sets, 2)),
                             sharey=True)
    for ax, (metric, label, scale) in zip(axes, BOX_METRICS):
        data = []
        for k in range(n_sets):
            vals = [get_path(r['metrics'], metric) for r in counted
                    if (r['record'].get('key') or [None])[0] == k]
            data.append([v * scale for v in vals if v is not None])
        positions = list(range(1, n_sets + 1))
        non_empty = [(p, d) for p, d in zip(positions, data) if d]
        if non_empty:
            bp = ax.boxplot([d for _, d in non_empty], positions=[p for p, _ in non_empty],
                            vert=False, widths=0.55, patch_artist=True, showfliers=True)
            for b in bp['boxes']:
                b.set(facecolor='#eeeeee', edgecolor=BLACK)
            for m in bp['medians']:
                m.set(color=BLACK, linewidth=1.6)
            for f in bp['fliers']:
                f.set(marker='o', markersize=3, markeredgecolor=GRAY)
        ax.set_title(label, fontsize=8)
        ax.set_yticks(positions)
        ax.set_yticklabels([s['name'][:28] for s in agg['sets']])
        ax.invert_yaxis()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def coverage_over_time(agg, path):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(7.5, 3.2))
    any_line = False
    for r in agg['counted']:
        series = ((r['metrics'] or {}).get('exploration') or {}).get('coverage_series') or []
        if not series:
            continue
        t = [p[0] / 60.0 for p in series]
        c = [p[1] * 100 for p in series]
        ax.plot(t, c, color=BLACK, alpha=0.25, linewidth=0.8)
        any_line = True
    ax.set_xlabel('sim time (min)')
    ax.set_ylabel('coverage (%)')
    ax.set_ylim(0, 102)
    ax.set_title('Coverage over time (one line per run)')
    if not any_line:
        ax.text(0.5, 0.5, 'no data', transform=ax.transAxes, ha='center', color=GRAY)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def drift_over_time(agg, path):
    plt = _plt()
    fig, ax = plt.subplots(figsize=(7.5, 3.2))
    colors = {'slam': BLUE, 'ekf': BLACK, 'wheels': GRAY}
    labels = {'slam': 'SLAM (map)', 'ekf': 'EKF (odom)', 'wheels': 'wheel odometry'}
    shown = set()
    for r in agg['counted']:
        series = ((r['metrics'] or {}).get('localization') or {}).get('series') or {}
        t = series.get('t') or []
        if not t:
            continue
        t0 = t[0]
        for name in ('wheels', 'ekf', 'slam'):
            vals = series.get(name)
            if not vals:
                continue
            pts = [((tt - t0) / 60.0, v * 100) for tt, v in zip(t, vals) if v is not None]
            if not pts:
                continue
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=colors[name], alpha=0.35,
                    linewidth=0.8, label=None if name in shown else labels[name])
            shown.add(name)
    ax.set_xlabel('sim time (min)')
    ax.set_ylabel('position error (cm)')
    ax.set_title('Position error against ground truth')
    if shown:
        ax.legend(frameon=False, fontsize=8)
    else:
        ax.text(0.5, 0.5, 'no data', transform=ax.transAxes, ha='center', color=GRAY)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


CHARTS = [('outcomes.png', 'Outcomes per maze set', outcomes_per_set),
          ('distributions.png', 'Main metrics per maze set', distributions),
          ('coverage.png', 'Coverage over time', coverage_over_time),
          ('drift.png', 'Localization error over time', drift_over_time)]


def draw_all(agg, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    made = []
    for name, title, fn in CHARTS:
        try:
            fn(agg, os.path.join(out_dir, name))
            made.append({'file': name, 'title': title})
        except Exception as exc:  # a chart must never stop the results
            made.append({'file': None, 'title': title, 'error': str(exc)})
    return made
