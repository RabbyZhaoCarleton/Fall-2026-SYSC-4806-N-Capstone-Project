"""Automatically find the RPLIDAR's USB device and hand it to WSL (usbipd-win).

WHY THIS EXISTS
---------------
On Windows + WSL2 the USB device belongs to Windows. It only shows up in Ubuntu after you run,
from PowerShell:   usbipd list   ->   usbipd attach --wsl --busid <BUSID>
The BUSID (e.g. "1-1") changes with the USB port / hub / computer, so it can't be hard-coded.
This module does that by hand-typed procedure automatically, from inside WSL:

  1. Call the Windows program usbipd.exe through WSL interop and parse `usbipd list`.
  2. Pick the LiDAR by USB VID:PID (default 10c4:ea60 = Silicon Labs CP210x, the C1's adapter),
     NOT by BUSID. An optional BUSID / description filter disambiguates when an ESP32 board
     (often the same CP210x chip) is also plugged in.
  3. If the device is "Not shared", `usbipd bind` it once (needs admin -> a UAC prompt appears).
  4. `usbipd attach --wsl` it, unless it is already attached.
  5. Wait for the serial port to appear in Ubuntu (/dev/rplidar from the udev rule, or the lone
     /dev/ttyUSB* if udev isn't running) and return its path.

On native Linux (the Raspberry Pi 5) none of that is needed: steps 1-4 are skipped and the
module only waits for the port, so the same code runs unchanged on the robot.

No ROS dependency: usable from the launch file, from a shell script, or from the command line:
    ros2 run lidar_bringup attach_lidar            # prints the port path on stdout
    ros2 run lidar_bringup attach_lidar --list     # show what usbipd sees (debugging)
"""
import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

# USB ID of the Silicon Labs CP210x bridge on the RPLIDAR C1 adapter (matches udev/99-rplidar.rules).
DEFAULT_VID_PID = "10c4:ea60"
# Symlink created by the udev rule; preferred over raw /dev/ttyUSBx names.
DEFAULT_PORT_LINK = "/dev/rplidar"

# usbipd-win installs here; used if usbipd.exe is not on the PATH WSL inherits from Windows.
_USBIPD_FALLBACK = "/mnt/c/Program Files/usbipd-win/usbipd.exe"
_POWERSHELL_FALLBACK = "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"

# A BUSID looks like "1-1" or "2-4.3". Validated before it is put into any command line.
_BUSID_RE = re.compile(r"^\d+-\d+(\.\d+)*$")

# One row of `usbipd list`:
#   1-1    10c4:ea60  Silicon Labs CP210x USB to UART Bridge (COM5)      Shared
# The description may contain spaces, so it is matched lazily up to the trailing state column.
_ROW_RE = re.compile(
    r"^(?P<busid>\d+-\d+(?:\.\d+)*)\s+"
    r"(?P<vid_pid>[0-9a-fA-F]{4}:[0-9a-fA-F]{4})\s+"
    r"(?P<desc>.+?)\s+"
    r"(?P<state>Not shared|Shared(?: \(forced\))?|Attached.*)$"
)

LogFn = Callable[[str], None]


def _default_log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


class LidarPortError(RuntimeError):
    """Raised when the LiDAR port can't be found/attached. The message says what to do."""


@dataclass
class UsbDevice:
    """One connected USB device as reported by `usbipd list`."""
    busid: str
    vid_pid: str       # lower-case "vvvv:pppp"
    description: str
    state: str         # "Not shared" | "Shared" | "Shared (forced)" | "Attached..."

    @property
    def is_attached(self) -> bool:
        return self.state.startswith("Attached")

    @property
    def is_shared(self) -> bool:
        # "Attached" implies it was shared first.
        return self.state.startswith("Shared") or self.is_attached


# --------------------------------------------------------------------------- environment helpers
def is_wsl() -> bool:
    """True when running inside WSL (kernel release string contains 'microsoft')."""
    try:
        with open("/proc/sys/kernel/osrelease") as f:
            return "microsoft" in f.read().lower()
    except OSError:
        return False


def find_usbipd() -> Optional[str]:
    """Path of usbipd.exe as callable from WSL, or None if usbipd-win isn't installed."""
    return shutil.which("usbipd.exe") or (
        _USBIPD_FALLBACK if os.path.exists(_USBIPD_FALLBACK) else None)


def _usbipd_env() -> dict:
    """Environment for usbipd.exe with WSL_UTF8 forced to 0.

    If the Windows variable WSL_UTF8=1 is set, usbipd crashes with "WSL failed to parse
    distributions" (it can't read wsl.exe's output). Adding the variable to WSLENV makes WSL
    forward our value 0 to the Windows process, overriding it. Harmless if it was never set.
    """
    env = dict(os.environ)
    env["WSL_UTF8"] = "0"
    names = [e.split("/")[0] for e in env.get("WSLENV", "").split(":") if e]
    if "WSL_UTF8" not in names:
        env["WSLENV"] = ":".join(filter(None, [env.get("WSLENV", ""), "WSL_UTF8"]))
    return env


def _decode(raw: bytes) -> str:
    """Windows programs may emit UTF-16 or UTF-8; decode either and normalise line endings."""
    if raw.startswith((b"\xff\xfe", b"\xfe\xff")) or b"\x00" in raw:
        text = raw.decode("utf-16", errors="replace")
    else:
        text = raw.decode("utf-8", errors="replace")
    return text.replace("\r\n", "\n").replace("\r", "\n")


def _run(cmd: List[str], timeout: float = 30.0) -> Tuple[int, str]:
    """Run a command, return (exit code, combined stdout+stderr text). Never raises on non-zero."""
    try:
        proc = subprocess.run(cmd, capture_output=True, timeout=timeout, env=_usbipd_env())
    except FileNotFoundError as exc:
        raise LidarPortError(f"Could not run '{cmd[0]}': {exc}") from exc
    except subprocess.TimeoutExpired as exc:
        raise LidarPortError(f"'{' '.join(cmd)}' did not finish within {timeout:.0f}s") from exc
    return proc.returncode, (_decode(proc.stdout) + _decode(proc.stderr)).strip()


# --------------------------------------------------------------------------- usbipd interaction
def parse_usbipd_list(text: str) -> List[UsbDevice]:
    """Turn the text of `usbipd list` into UsbDevice objects.

    Only the "Connected:" table is read; parsing stops at the "Persisted:" table (devices that
    are remembered but not currently plugged in).
    """
    devices: List[UsbDevice] = []
    for line in text.splitlines():
        line = line.strip()
        if line.lower().startswith("persisted"):
            break
        m = _ROW_RE.match(line)
        if m:
            devices.append(UsbDevice(m["busid"], m["vid_pid"].lower(),
                                     m["desc"].strip(), m["state"].strip()))
    return devices


def list_devices(usbipd: str) -> List[UsbDevice]:
    rc, out = _run([usbipd, "list"])
    if rc != 0:
        raise LidarPortError(f"'usbipd list' failed (exit {rc}):\n{out}")
    return parse_usbipd_list(out)


def _describe(devices: List[UsbDevice]) -> str:
    return "\n".join(f"    {d.busid:<6} {d.vid_pid}  {d.description}  [{d.state}]"
                     for d in devices) or "    (no USB devices reported)"


def select_lidar(devices: List[UsbDevice], vid_pid: str = DEFAULT_VID_PID,
                 busid: Optional[str] = None,
                 description_contains: Optional[str] = None) -> UsbDevice:
    """Choose which connected device is the LiDAR.

    Priority: an explicit `busid` wins; otherwise filter by VID:PID (and optionally by a
    substring of the description). Exactly one match is required - if two devices match
    (e.g. LiDAR + an ESP32 with the same CP210x chip) we refuse to guess, because attaching
    the wrong one would hand the ESP32 to WSL instead.
    """
    if busid:
        for d in devices:
            if d.busid == busid:
                return d
        raise LidarPortError(f"No USB device with BUSID {busid} is connected. usbipd sees:\n"
                             f"{_describe(devices)}")

    matches = [d for d in devices if d.vid_pid == vid_pid.lower()]
    if description_contains:
        matches = [d for d in matches if description_contains.lower() in d.description.lower()]

    if not matches:
        raise LidarPortError(
            f"No USB device with ID {vid_pid} is connected. Check the LiDAR's USB cable "
            f"(and that it is powered). usbipd sees:\n{_describe(devices)}")
    if len(matches) > 1:
        raise LidarPortError(
            f"{len(matches)} USB devices match {vid_pid}, so I won't guess which is the LiDAR:\n"
            f"{_describe(matches)}\nPick one with lidar_busid:=<BUSID> (launch) / --busid <BUSID>, "
            "or narrow it with lidar_description:=<text from the description>.")
    return matches[0]


def _check_busid(busid: str) -> None:
    if not _BUSID_RE.match(busid):
        raise LidarPortError(f"Refusing to use malformed BUSID '{busid}'")


def ensure_bound(usbipd: str, dev: UsbDevice, auto_bind: bool, log: LogFn) -> None:
    """Make sure the device is 'Shared' (bound). Binding needs administrator rights, once per device."""
    if dev.is_shared:
        return
    _check_busid(dev.busid)
    log(f"[usb] {dev.busid} is not shared yet; binding it (one-time step per device)")
    rc, out = _run([usbipd, "bind", "--busid", dev.busid])
    if rc == 0:
        return

    manual = f"Run in an ADMIN PowerShell:  usbipd bind --busid {dev.busid}"
    if not auto_bind:
        raise LidarPortError(f"'usbipd bind' needs administrator rights ({out}).\n{manual}")

    # No admin rights from inside WSL: ask Windows to run the bind elevated (shows a UAC prompt).
    log("[usb] bind needs administrator rights -> accept the Windows UAC prompt")
    ps = shutil.which("powershell.exe") or _POWERSHELL_FALLBACK
    script = (f"Start-Process -FilePath 'usbipd' -ArgumentList 'bind','--busid','{dev.busid}' "
              "-Verb RunAs -Wait")
    rc, out = _run([ps, "-NoProfile", "-Command", script], timeout=120.0)

    # Elevation can be cancelled silently, so verify the real state instead of trusting rc.
    for d in list_devices(usbipd):
        if d.busid == dev.busid and d.is_shared:
            return
    raise LidarPortError(f"Could not bind {dev.busid} (UAC declined or failed). {manual}\n{out}")


def attach(usbipd: str, dev: UsbDevice, log: LogFn) -> None:
    """`usbipd attach --wsl --busid X` for *this* WSL distribution."""
    _check_busid(dev.busid)
    cmd = [usbipd, "attach", "--wsl"]
    distro = os.environ.get("WSL_DISTRO_NAME")   # name the distro explicitly; the Windows
    if distro:                                   # default distro may be a different one
        cmd.append(distro)
    cmd += ["--busid", dev.busid]
    log(f"[usb] attaching {dev.busid} ({dev.description}) to WSL")
    rc, out = _run(cmd)
    if rc == 0 or "already attached" in out.lower():
        return
    hint = ""
    if "failed to parse" in out.lower():
        hint = "\nThe Windows variable WSL_UTF8=1 is set: clear it (PowerShell: $env:WSL_UTF8 = $null)."
    raise LidarPortError(f"'usbipd attach' failed (exit {rc}):\n{out}{hint}")


# --------------------------------------------------------------------------- serial port discovery
def _tty_fallback() -> Optional[str]:
    """The single /dev/ttyUSB* if exactly one exists (used when the udev symlink is missing)."""
    ttys = sorted(glob.glob("/dev/ttyUSB*"))
    return ttys[0] if len(ttys) == 1 else None


def wait_for_port(timeout: float, port_link: str = DEFAULT_PORT_LINK,
                  log: LogFn = _default_log) -> Optional[str]:
    """Poll until the LiDAR's serial port exists; None on timeout.

    The kernel creates /dev/ttyUSBx a moment after the attach, and udev then adds the
    /dev/rplidar symlink. We wait for the symlink; if it never appears but exactly one
    ttyUSB exists, we use that and say why (udev isn't active in this WSL instance).
    """
    deadline = time.monotonic() + timeout
    tty_seen_at: Optional[float] = None
    while time.monotonic() < deadline:
        if os.path.exists(port_link):
            return port_link
        if _tty_fallback() and tty_seen_at is None:
            tty_seen_at = time.monotonic()
        # Give udev 3 s to create the symlink after the tty shows up before falling back.
        if tty_seen_at is not None and time.monotonic() - tty_seen_at > 3.0:
            tty = _tty_fallback()
            log(f"[usb] {port_link} did not appear; using {tty}. The udev rule is not active "
                "here (see README: install_udev.sh, and systemd for WSL).")
            return tty
        time.sleep(0.25)
    return None


def ensure_lidar_port(vid_pid: str = DEFAULT_VID_PID, busid: Optional[str] = None,
                      description_contains: Optional[str] = None, auto_bind: bool = True,
                      timeout: float = 15.0, port_link: str = DEFAULT_PORT_LINK,
                      log: LogFn = _default_log) -> str:
    """Do whatever is needed so the LiDAR has a usable serial port; return that port's path.

    Raises LidarPortError (with an actionable message) if that is not possible.
    """
    if not is_wsl():
        # Native Linux / Raspberry Pi: nothing to attach, just wait for the device node.
        port = wait_for_port(timeout, port_link, log)
        if port is None:
            raise LidarPortError(
                f"{port_link} did not appear within {timeout:.0f}s. Check the USB cable, "
                "`dmesg | tail` for a cp210x message (Ubuntu on Pi 5 needs "
                "linux-modules-extra-raspi) and that udev/99-rplidar.rules is installed.")
        return port

    # Fast path: already attached from an earlier run, nothing to do.
    if os.path.exists(port_link):
        log(f"[usb] {port_link} already present")
        return port_link

    usbipd = find_usbipd()
    if usbipd is None:
        raise LidarPortError(
            "usbipd.exe not found from WSL. Install it once from Windows PowerShell: "
            "winget install usbipd   (then restart the WSL terminal).")

    dev = select_lidar(list_devices(usbipd), vid_pid, busid, description_contains)
    log(f"[usb] LiDAR = BUSID {dev.busid}: {dev.description} [{dev.state}]")

    if not dev.is_attached:
        ensure_bound(usbipd, dev, auto_bind, log)
        attach(usbipd, dev, log)

    port = wait_for_port(timeout, port_link, log)
    if port is None:
        raise LidarPortError(
            f"Attached {dev.busid} but no serial port appeared within {timeout:.0f}s. "
            "Run `dmesg | tail` in WSL: if there is no cp210x line, your WSL kernel lacks the "
            "CP210x USB-serial driver (update with `wsl --update`).")
    return port


# --------------------------------------------------------------------------- command line
def main(argv: Optional[List[str]] = None) -> int:
    p = argparse.ArgumentParser(description="Find and attach the RPLIDAR USB device (WSL), "
                                            "then print its serial port path.")
    p.add_argument("--vid-pid", default=DEFAULT_VID_PID, help="USB ID to look for")
    p.add_argument("--busid", help="force a BUSID instead of detecting by VID:PID")
    p.add_argument("--description-contains", help="extra filter on the device description")
    p.add_argument("--no-auto-bind", action="store_true",
                   help="don't request administrator elevation for the one-time bind")
    p.add_argument("--timeout", type=float, default=15.0, help="seconds to wait for the port")
    p.add_argument("--port-link", default=DEFAULT_PORT_LINK)
    p.add_argument("--list", action="store_true", help="just print what usbipd sees")
    a = p.parse_args(argv)

    try:
        if a.list:
            usbipd = find_usbipd()
            if usbipd is None:
                raise LidarPortError("usbipd.exe not found (WSL only)")
            print(_describe(list_devices(usbipd)))
            return 0
        port = ensure_lidar_port(a.vid_pid, a.busid, a.description_contains,
                                 not a.no_auto_bind, a.timeout, a.port_link)
    except LidarPortError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    print(port)            # stdout = just the path, so scripts can do PORT=$(attach_lidar)
    return 0


if __name__ == "__main__":
    sys.exit(main())
