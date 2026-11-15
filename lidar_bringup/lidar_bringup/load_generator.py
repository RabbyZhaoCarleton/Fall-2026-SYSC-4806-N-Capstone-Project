"""Burn CPU on N cores to emulate SLAM/Nav2/vision load while you watch /scan health.

  ros2 run lidar_bringup load_generator --cores 3 --duration 60

Uses multiprocessing (separate processes) so the Python GIL does not cap the load.
For a closer-to-real test also run `stress-ng --cpu 3 --timeout 60s` or the real slam_toolbox.
"""
import argparse
import multiprocessing as mp
import time


def _burn(stop_at: float) -> None:
    x = 0.0
    while time.perf_counter() < stop_at:
        for i in range(10_000):
            x += i * 0.5 ** 0.5


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cores", type=int, default=max(1, mp.cpu_count() - 1))
    p.add_argument("--duration", type=float, default=30.0, help="seconds")
    a = p.parse_args()
    print(f"Loading {a.cores} core(s) for {a.duration:.0f}s ...")
    stop_at = time.perf_counter() + a.duration
    procs = [mp.Process(target=_burn, args=(stop_at,)) for _ in range(a.cores)]
    for pr in procs:
        pr.start()
    for pr in procs:
        pr.join()
    print("Load finished.")


if __name__ == "__main__":
    main()
