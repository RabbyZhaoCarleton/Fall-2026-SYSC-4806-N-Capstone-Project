"""Open folder, Open terminal here, Copy path (2.3.4, GR4).

A standard WSL Ubuntu install has no Linux terminal emulator, and Nautilus is
not always installed, so each action falls back to what exists.
"""

import os
import shutil
import subprocess


def _detached(cmd, cwd=None):
    subprocess.Popen(cmd, cwd=cwd, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                     stderr=subprocess.DEVNULL, start_new_session=True)


def windows_path(path):
    try:
        return subprocess.run(['wslpath', '-w', path], capture_output=True, text=True,
                              timeout=10).stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def _windows_exe(name, fallback):
    return shutil.which(name) or (fallback if os.path.exists(fallback) else None)


def open_folder(path):
    """Nautilus if installed, else Windows Explorer with the path from wslpath -w.

    Returns a short description of what happened."""
    if shutil.which('nautilus'):
        _detached(['nautilus', '--new-window', path])
        return 'Opened in Nautilus'
    explorer = _windows_exe('explorer.exe', '/mnt/c/Windows/explorer.exe')
    win = windows_path(path)
    if explorer and win:
        _detached([explorer, win])
        return 'Opened in Windows Explorer'
    return None


LINUX_TERMINALS = (
    ('gnome-terminal', lambda p: ['gnome-terminal', f'--working-directory={p}']),
    ('konsole', lambda p: ['konsole', '--workdir', p]),
    ('xfce4-terminal', lambda p: ['xfce4-terminal', f'--working-directory={p}']),
    ('tilix', lambda p: ['tilix', '-w', p]),
    ('x-terminal-emulator', lambda p: ['x-terminal-emulator']),
    ('xterm', lambda p: ['xterm']),
)


def open_terminal(path):
    """Windows Terminal in this WSL distribution, else a Linux terminal, else None."""
    wt = _windows_exe('wt.exe', '')
    distro = os.environ.get('WSL_DISTRO_NAME')
    if wt and distro:
        _detached([wt, '-w', 'new', 'wsl.exe', '-d', distro, '--cd', path])
        return 'Opened Windows Terminal'
    for exe, args in LINUX_TERMINALS:
        if shutil.which(exe):
            _detached(args(path), cwd=path)
            return f'Opened {exe}'
    return None
