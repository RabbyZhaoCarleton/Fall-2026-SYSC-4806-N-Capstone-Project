"""The color palette. Each color has one meaning everywhere in the monitor.

    Red     error or danger (collision, contact, failed goal)
    Amber   warning (close to a wall, stuck, slow simulation)
    Green   OK, goal reached
    Blue    intent: what the robot is told or plans (commands, path, goal)
    Black   reality: what the robot measured or did (odometry, pose, trail)
    Gray    no data, stale data, or unknown map area
    Purple  exploration frontiers (green ring: maze exit, dashed: probable exit;
            gray: postponed, amber: abandoned)
"""

from python_qt_binding.QtGui import QColor, QFont, QFontDatabase, QPalette

RED = QColor('#c62828')
AMBER = QColor('#b26a00')
GREEN = QColor('#2e7d32')
BLUE = QColor('#1565c0')
BLACK = QColor('#212121')
GRAY = QColor('#8a8a8a')
LIGHT = QColor('#d8d8d8')
FAINT = QColor('#f0f0f0')
PURPLE = QColor('#6a1b9a')
WHITE = QColor('#ffffff')

SEVERITY = {'info': GRAY, 'warning': AMBER, 'error': RED}
# frontier states from explore_lite
FRONTIER = {'open': PURPLE, 'candidate': GREEN, 'exit': GREEN, 'postponed': GRAY,
            'abandoned': AMBER}
LEVEL = {'ok': GREEN, 'warning': AMBER, 'error': RED}


def alpha(color, a):
    c = QColor(color)
    c.setAlpha(a)
    return c


def mono_font(size_delta=0):
    f = QFontDatabase.systemFont(QFontDatabase.FixedFont)
    if size_delta:
        f.setPointSizeF(max(f.pointSizeF() + size_delta, 6))
    return f


def bold(font=None):
    f = QFont(font) if font is not None else QFont()
    f.setBold(True)
    return f


def light_palette():
    """A fixed light palette, so map colors (white free, black walls) always read the same."""
    p = QPalette()
    p.setColor(QPalette.Window, QColor('#f5f5f5'))
    p.setColor(QPalette.WindowText, BLACK)
    p.setColor(QPalette.Base, WHITE)
    p.setColor(QPalette.AlternateBase, QColor('#f7f7f7'))
    p.setColor(QPalette.Text, BLACK)
    p.setColor(QPalette.Button, QColor('#ececec'))
    p.setColor(QPalette.ButtonText, BLACK)
    p.setColor(QPalette.Highlight, QColor('#cfd8e3'))
    p.setColor(QPalette.HighlightedText, BLACK)
    p.setColor(QPalette.ToolTipBase, WHITE)
    p.setColor(QPalette.ToolTipText, BLACK)
    p.setColor(QPalette.Disabled, QPalette.Text, GRAY)
    p.setColor(QPalette.Disabled, QPalette.WindowText, GRAY)
    p.setColor(QPalette.Disabled, QPalette.ButtonText, GRAY)
    return p


def fmt_time(t):
    if t is None:
        return '—'
    t = max(t, 0.0)
    m, s = divmod(t, 60.0)
    return f'{int(m):02d}:{s:04.1f}'


def fmt_bytes(n):
    for unit in ('B', 'KB', 'MB', 'GB'):
        if abs(n) < 1024 or unit == 'GB':
            return f'{n:.0f} {unit}' if unit == 'B' else f'{n:.1f} {unit}'
        n /= 1024.0
