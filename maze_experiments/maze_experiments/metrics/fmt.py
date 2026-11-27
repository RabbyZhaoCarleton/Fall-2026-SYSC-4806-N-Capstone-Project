"""Values for people (ER7): SI units with sensible precision, degrees, m:ss durations."""

import math


def duration(s):
    if s is None:
        return '—'
    s = float(s)
    sign = '-' if s < 0 else ''
    s = abs(s)
    if s >= 3600:
        h, rem = divmod(int(round(s)), 3600)
        return f'{sign}{h}:{rem // 60:02d}:{rem % 60:02d}'
    m, sec = divmod(int(round(s)), 60)
    return f'{sign}{m}:{sec:02d}'


def percent(v, digits=0):
    if v is None:
        return '—'
    return f'{v * 100:.{digits}f}%'


def value(v, kind, unit=''):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return '—'
    if kind == 'duration':
        return duration(v)
    if kind == 'percent':
        return percent(v, 1)
    if kind == 'ratio':
        return f'{v:.2f}'
    if kind == 'count':
        return f'{v:.1f}' if isinstance(v, float) and v != int(v) else f'{int(round(v))}'
    if kind == 'angle':
        return f'{v:.1f}°'
    if kind == 'small_length':
        return f'{v * 100:.1f} cm' if abs(v) < 1.0 else f'{v:.2f} m'
    if kind == 'length':
        return f'{v:.2f} m'
    return f'{v:.3g}{" " + unit if unit else ""}'


def rate_text(r, digits=0):
    """'92 of 100 (92%, 95% CI 85–96%)'."""
    if not r or not r.get('n'):
        return 'no runs'
    low = f'{r["ci_low"] * 100:.{digits}f}'
    return (f'{r["k"]} of {r["n"]} ({percent(r["share"], digits)}, 95% CI '
            f'{low}–{percent(r["ci_high"], digits)})')


def ci_text(r):
    if not r or not r.get('n'):
        return '—'
    return f'{percent(r["share"])} ({r["ci_low"] * 100:.0f}–{percent(r["ci_high"])})'


def size(n):
    if n is None:
        return 'unknown'
    for unit in ('B', 'KB', 'MB', 'GB', 'TB'):
        if abs(n) < 1024 or unit == 'TB':
            return f'{n:.0f} {unit}' if unit == 'B' else f'{n:.1f} {unit}'
        n /= 1024.0


def long_duration(s):
    """'4 min 36 s', '2 h 05 min': for estimates."""
    if s is None:
        return '—'
    s = int(round(float(s)))
    if s >= 3600:
        return f'{s // 3600} h {(s % 3600) // 60:02d} min'
    if s >= 60:
        return f'{s // 60} min {s % 60:02d} s'
    return f'{s} s'
