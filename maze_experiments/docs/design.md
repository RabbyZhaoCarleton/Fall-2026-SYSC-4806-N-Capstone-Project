# Maze experiments design

This document describes the architecture of `maze_experiments`, the test suite for
the maze robot, and explains why it is built the way it is. Requirement IDs (AR1,
BR9, ER2 and so on) refer to the specification in the workspace
(`~/ros2_ws/Specifications.md`). For how to use the suite, see the
[user manual](user-manual.md).

## Goals

The suite answers one question: how well does the robot's software stack explore a
maze, find its exits and return to the start, across many mazes? It must:

*   generate random mazes from parameters and a seed, as Gazebo worlds that
    `maze.launch.py` loads unchanged (AR1, AR8);
*   run the real simulated stack on many mazes at once, without the runs disturbing
    each other (BR1, BR2), and keep them close to real time (Module C);
*   record every run in the maze monitor's format, so any run can be replayed (DR1);
*   survive crashes, kills and WSL restarts, keeping finished runs and repeating
    interrupted ones (BR12, DR7, DR8);
*   turn recordings into metrics, aggregates with confidence intervals and findings,
    in a GUI and on the command line (Module E, GR1);
*   record which code and parameters produced each result (DR6).

### Non-goals

*   Controlling or tuning the robot stack. The suite selects parameter files for a
    run and never edits the repository (1.2).
*   Running on the physical robot, or replacing `maze_monitor` or `ros2 bag`. The
    suite reuses the monitor's recording and analysis code and writes rosbag2 files.
*   Running on more than one PC or user account. Everything runs without root (S3).

## Overview

The suite runs three kinds of process. The GUI and the command line only write files
and start a manager. A manager is a background process for one experiment. Each run
has its own process group, led by a small run agent.

```
 GUI (maze_experiments)                  CLI (maze_experiments <command>)
   home, editor, run dialog,               config, run, status, pause, ...
   run screen, results screen
          |                                         |
          +------------------+----------------------+
                             |  both call operations.Suite: write the configuration
                             |  or experiment folder, then start a detached manager
                             v
 +--------------------------------------------------------------------------+
 | Manager process, one per experiment (manager.py)                         |
 |   flock on experiments/<id>/manager.lock, held for its whole life       |
 |   every 1 s: control.jsonl, clock checks, machine sample, each run's     |
 |   status.jsonl, outcome decisions, stops and starts, CR7 protection,     |
 |   processing; writes manager.jsonl, live.json, resources.jsonl          |
 +------------------------------+-------------------------------------------+
                                | starts one run; reads only that run's files
                                | env: ROS_DOMAIN_ID, ROS_AUTOMATIC_DISCOVERY_RANGE,
                                |      GZ_PARTITION, GZ_IP, MAZE_EXPERIMENTS_RUN
                                v
 +--------------------------------------------------------------------------+
 | Run process group (own session; leader = run agent, agent.py)            |
 |   _agent: RosBridge + Recorder (maze_monitor) -> runs/<id>/bag           |
 |           status.jsonl every 1 s                                         |
 |   +-- ros2 launch maze_sim maze.launch.py monitor:=false ...             |
 |         Gazebo server, ROS-Gazebo bridge, SLAM, Nav2, explore_lite,      |
 |         ground-truth bridge                                              |
 +--------------------------------------------------------------------------+
                                |
 after a run, at lower priority: nice -n 15 python -m maze_experiments.main
                                 _process-run <run dir>  -> metrics.json
```

| Process | Started by | Lives until | Writes |
|---------|-----------|-------------|--------|
| GUI, CLI command | the user | the window closes, or the command ends | only through `Suite` |
| Manager | `Suite.start` or `resume`, in a new session | the experiment ends, is cancelled or is interrupted | `manager.jsonl`, `live.json`, `resources.jsonl`, `experiment.json`, run outcomes |
| Run agent | the manager, in its own session | the launch has exited and the recording is closed | `runs/<id>/status.jsonl`, `bag/`, `agent.log` |
| Launch | the agent, in the same group | the simulation ends or is stopped | `launch.log` |
| Processing | the manager, `nice -n 15` | the metrics are written | `metrics.json` |

**Why:** the manager is a separate process, so closing the GUI or the terminal does
not stop an experiment (BNR5). The GUI never runs an experiment itself.

**Why:** the agent leads the run's process group. The manager stops a run with SIGINT
to the group, as Ctrl+C would. The launch shuts the stack down, and the agent closes
the recording (BR2).

**Why:** the manager reads only files in the run folder, never a run's ROS graph. It
never joins a run's ROS domain, and the files double as the run's log (BR8).

## Components

The package's modules map to the modules of the specification as follows:

| Module | Files in `maze_experiments/` |
|--------|------------------------------|
| Shared | `main.py`, `cli.py`, `operations.py` (`Suite`), `configuration.py`, `experiment.py`, `profiles.py`, `fileio.py`, `paths.py`, `ids.py`, `recording.py`, `provenance.py` |
| A: generation | `mazegen/` (generator, parameters, sets, files, preview) |
| B: manager | `manager.py`, `manager_control.py`, `agent.py`, `leases.py` |
| C: resources | `resources.py` |
| E: results | `metrics/` (run metrics, geometry, aggregation, findings, results, charts, message text) |
| G: interface | `gui/` (home, editor, run dialog, run and results screens) |

Tests are in `test/`, and the robot profile, recording rules and example are in `config/`.
The suite depends on `maze_monitor` (`RosBridge`, `Recorder`, `RunStore`, `Session`, analyzers,
the style module) and on `maze_sim` (`maze.launch.py` and the parameter files, through
`package://` paths). It changes nothing else in `maze_sim` or `maze_navigation` except the launch
arguments listed in 1.2.

## Experiment states and configurations

The state is kept in `experiment.json` (FR5), and each change is appended to
`state_history`. The transitions are:

*   `created` to `generating`: the manager starts and a maze set lacks its mazes.
*   `generating` (or `interrupted`, on resume) to `calibrating`, or `running` when a
    usable calibration exists or none is needed.
*   `calibrating` to `running`: once the calibration run is measured.
*   `calibrating` or `running` to `paused`, and back: pause and resume, or the disk
    protection (reason `disk`).
*   Active states to `processing`, then `completed` when the results are built. A
    cancel request ends in `cancelled`, after the active runs are stopped as interrupted.
*   Active states to `interrupted`: the manager got SIGINT or SIGTERM, hit an error,
    or is found gone by the next command or GUI start.

`Suite.find_orphans` turns an active experiment with no live manager into an
interrupted one, after stopping its leftover runs. The command line does this before
every command except `config` and `machine`, and the GUI does it at start-up.

**Configurations.** A configuration is YAML in `configurations/<id>/`, with copies of
its parameter files in `params/` (FR8). It is never edited. Save always creates one,
and "Create modified version" copies the fields into a new one (2.3.3). **Why:** an
experiment copies its configuration, so an edited one would no longer match what ran.

**Sweeps (FR3).** `configuration.expand_sets` makes one set per value of each list-valued
maze parameter, and one per combination when several are lists. `start_cell` sweeps only
as a list of cells.

**Run plan (BR14, FNR1).** `experiment.plan_runs` orders runs round-robin: for each
repeat, each maze index, each set in turn. A partial experiment then covers every set
evenly, and slow drifts in load cannot line up with one set. The order is deterministic.

**IDs (DR3).** Configuration and experiment IDs are a timestamp, a four-hex-digit random
suffix and a slug of the name. A run ID is `s<set>-m<maze>-r<repeat>-a<attempt>`, so a
resume finds the same folders again.

**Checks (FR4).** `configuration.check` reports every problem at once: each set against
Module A's rules, the robot profile and its parameter files, the workspace (`maze_sim` and
`maze_experiments` installed), and the data directory (writable, not under `/mnt`). The
estimates are checked against the allocation in `Suite.prepare_run`.

## Module B: the run manager

### Process and lock

`Manager` runs as `python -m maze_experiments.main _manager <experiment>` in a new
session, with output appended to `manager.out`. It takes an exclusive `flock` on
`manager.lock` for its whole life. "Is a manager alive?" is a shared-lock attempt on that
file. The kernel releases the lock however the process ends, so a stale PID file never
makes a dead manager look alive.

### The tick

Every second the manager does, in order:

1.  Read new lines of `control.jsonl` (pause, resume, cancel).
2.  Check for a host suspend or clock jump (BR13).
3.  Sample the machine, and read each run's `status.jsonl` (BR8).
4.  Decide each running run's outcome, and stop the runs that must stop (BR3, BR5 to BR7).
5.  Advance the stops, and finish the runs that have stopped (BR2, BR4).
6.  Append to `resources.jsonl`, and update the concurrency controller (CR4, CR5, CR8).
7.  Protect the machine (CR7).
8.  Process finished runs (ER4), start runs if allowed, and write `live.json` (every 2 s)
    and the experiment summary (every 10 s).

The loop ends when nothing is queued, active, leftover or waiting for processing. The
manager then builds the results and sets the final state. SIGINT or SIGTERM stops the runs
as `interrupted` (reason `manager_stopped`), and sets the experiment to `interrupted`.

### Queue and attempts

`build_queue` queues one entry per planned run with no robot result, while its non-robot
failures are within the retry budget. The attempt number is one more than the highest on
record. The queue is rebuilt on every start, so a resume needs no state beyond the run folders.

### Starting a run

`Manager.start_run` acquires a domain lease and a Gazebo partition (see Isolation). If none
is free, the run waits. It then writes `run_spec.json` (identity, arguments, domain,
partition, confinement, boot ID) and starts the agent with the confinement prefix (Module C).
The launch arguments are `explore:=true`, `shutdown_when_done:=true`, `gui:=false`,
`rviz:=false`, `monitor:=false`, `gz_server_check:=partition`, the `world`, the spawn pose from
`maze.json`, the profile's launch arguments and spawn height, and the stack parameter files
(a set's own files override the profile's). After the start, the group ID and the leader's
start time go into the spec, and the lease is attached to the group.

The run's environment is the manager's, with `ROS_LOCALHOST_ONLY` removed and:

| Variable | Value | Why |
|----------|-------|-----|
| `ROS_DOMAIN_ID` | the leased domain, 1 to 101 | Runs cannot see each other's topics (BR1, BNR2). |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `LOCALHOST` | No run reaches the network or a teammate's robot (BR1). |
| `GZ_PARTITION` | `maze_exp_<domain>_<6 hex>` | Gazebo transport is separate for each run (BR1). |
| `GZ_IP` | `127.0.0.1` | Gazebo transport stays on loopback (BR1). |
| `MAZE_EXPERIMENTS_RUN` | `<experiment>/<run>` | A marker that finds the run's processes (see Stopping). |

The profile's `env` is added too.

### Readiness (BR3)

A run is ready when four milestones hold, read by the agent from its own domain:
`/clock` has advanced, `/scan` has published, `/ground_truth/poses` has published, and
`/explore/status` has reported `exploration_started`. The manager records the readiness
time and the stall reference (simulation time, map area and true position).

### Deciding a run's outcome (BR5 to BR7, BR9)

`Manager.evaluate` runs once per tick for each running run. The first condition that holds
decides:

| # | Condition | Decision |
|--:|-----------|----------|
| 1 | The agent wrote its final record, or the agent is gone, or the launch exited. If the final record is missing while the agent lives, the manager waits. If the agent is gone without one: `infra_failure` (`agent_exited`). Otherwise `outcome_from_statuses` decides. | see the next table |
| 2 | Not ready, and wall time exceeds the startup timeout. | `startup_failed`, "not ready: <milestones>" |
| 3 | Simulation time reaches the run timeout (BR5). | `timeout` |
| 4 | Wall time exceeds startup timeout + run timeout ÷ minimum RTF (BR6). | `sim_frozen` |
| 5 | A stack process died, not after a stop or launch shutdown, and no `returned_to_origin` or `return_failed` was seen. | `stack_crash` |
| 6 | Stall (BR7). The reference resets when the map area has grown by the minimum, or the true position has moved by the minimum, since the reference. Otherwise, when the simulation time since the reference reaches the window. | `stalled` |
| 7 | No systemd memory limit, and memory above 1.2 times the run's limit. | `infra_failure` (`memory_limit`) |

A decision goes through `stop`. A run that ended on its own is "stopped" with the outcome it
earned, and the signal finds nothing to stop.

`outcome_from_statuses` decides a run that ended on its own, from the explore statuses:

| Condition, in order | Outcome | Reason |
|---------------------|---------|--------|
| Agent gone, no final record. | `infra_failure` | `agent_exited` |
| `exploration_incomplete` seen. | `incomplete` | with or without "returned to the start", or "return failed" |
| `exploration_complete`, then `returned_to_origin`. | `complete` | `exploration_complete, returned_to_origin` |
| `exploration_complete`, then `return_failed`. | `return_failed` | `exploration_complete, return_failed` |
| `exploration_complete`, with no return status. | `stack_crash` | explore_lite ended without a final return status |
| Ready, and none of the above. | `stack_crash` | a process died, or no final explore status |
| Not ready. | `startup_failed` | the launch ended before the run was ready |

In `finish`, `bag_state` checks the recording once more. A robot result whose recording is
missing or unreadable becomes `infra_failure`. **Why:** an incomplete exploration is a robot
result, because the robot stopped on its own judgement (BR9). A stop caused by the PC is not.

### Stopping (BR2, BR4)

`Manager.stop` records the outcome and sends SIGINT to the group, and to any process that has
left the group but still carries the run's marker. `advance_stop` then escalates:

| Stage | Action | Wait before next stage |
|------:|--------|------------------------|
| 1 | SIGINT to the group | 20 s, for the recording to close |
| 2 | SIGTERM | 5 s |
| 3 | SIGKILL | 5 s |
| 4 | Log an error naming the surviving PIDs. The run keeps its domain and partition until they are gone. | polled |

A run has stopped when its leader is gone and no marked process with the same leader start time
remains. Processes are never stopped by name: `pkill -f` would hit another experiment or the
user's own simulation. The marker lives in `/proc/<pid>/environ`, which survives a process
leaving the group, and the start-time check keeps a reused PID safe.

### Finishing a run

`Manager.finish` checks the recording and sets the outcome. It writes the run's summary into
`run.json`: simulation and wall time, RTF mean and minimum, `low_rtf`, explore statuses, dead
processes, flags, CPU and memory, concurrency, and the calibration flag. It releases the lease,
runs `fastdds shm clean` (BR16), and logs `stopped` and `outcome`. The calibration run calibrates
(Module C). Then the run is repeated or not (below), and its folder is queued for processing (ER4).

### Retries (BR9, BR10)

Robot outcomes are never repeated. Non-robot outcomes are repeated up to `retries` times, counted
per planned run. The count excludes attempts whose reason is `manager_stopped`, `cancelled` or
`manager_crashed`, so an attempt the suite interrupted costs no retry. A run is repeated while the
count is at most `retries`. The same rule applies in `finish` and in `build_queue`.

**Why:** retrying robot failures would hide them and inflate the success rate (BR10). An
interruption the suite caused says nothing about the robot.

### Recovery (BR12)

At start-up, `recover` handles each attempt with no outcome: it was active when the manager
stopped. `kill_leftovers` stops its marked processes, after checking that the boot ID and leader
start time match, with the same escalation as a run. The attempt is finalized as `interrupted`
(reason `manager_crashed`), and queued for processing. `build_queue` then queues each planned run
without a robot result, with its next attempt number. Finished runs are kept, interrupted runs run
again, and nothing is duplicated.

A killed manager does not kill its runs, because they are in their own session. Recovery, or the
next command or GUI start, is what stops them.

### Host suspend and clock jumps (BR13, FR6)

Each tick compares the monotonic and boot clocks, and the wall clock, with the previous tick. A gap
of more than 30 s in either clock is a `host_suspend`. A wall-clock step that differs from the
monotonic step by more than 2 s is a `clock_jump`. Every 10 s the manager also asks the journal for
"Clock change detected" entries. Each event is logged as `host_event`.

A host suspend always stops the active runs as `infra_failure`, and they are repeated. For a clock
jump, the default `clock_jump_action: retry` does the same. The opt-in `flag` action (set by
`--clock-jumps flag` or the run dialog) only adds a flag to the run's record, and the run goes on.

**Deviation:** BR13 says runs active during a jump are marked `infra_failure` and queued again.
That is `retry`, the default. `flag` is an opt-in departure, chosen by the user.

Before an experiment, `resources.clock_warnings` warns when `systemd-timesyncd` is active under
WSL, or when the journal has clock-change entries in the last 10 minutes (FR6).

### Isolation (BR1, BR4, BNR2)

**Domain IDs (`leases.py`).** A lease is the file `~/.cache/maze_experiments/leases/domain-<n>.json`,
for n from 1 to 101 (0 is left for interactive use). It holds the domain, partition, run key, owner
PID, group, leader start time and boot ID. It is created with `O_EXCL`, so two managers cannot take
one ID. Until the group exists, the owner's PID keeps the lease alive. A lease is stale when its boot
ID differs, or no process of its group still carries its marker, and a stale lease is removed. A
lease with live processes is never reused, even if its manager died (BR4).

**Gazebo partitions.** Each name includes the domain and a random part. With
`gz_server_check:=partition`, the launch's "server already running" check ignores servers in other
partitions, and counts a server whose environment cannot be read as a match. The default `all` keeps
the launch's check unchanged (S5). **Why:** a leftover server publishes its own `/clock`, `/odom` and
`/scan`, so only a server in the same partition can disturb a run.

**Processes.** Finding a run's processes reads `/proc/<pid>/stat` and `/proc/<pid>/environ`, and skips
the manager itself. The suite is Linux-specific, as it must be on WSL.

### Logs and files (BR8, BR11, DR7)

Growing files are JSON lines with a header `{"schema_version": 1, "type": "header", "kind": ...}`.
Each record is one `write` in append mode, so writers never interleave within a line, and a reader
ignores a torn last line.

| File | Written by | Rate | Content |
|------|-----------|------|---------|
| `manager.jsonl` | manager | on events | `created`, `generating`, `maze_discarded`, `manager_started`, `queued`, `control`, `started` (domain, partition, group, CPUs, memory limit, confinement), `ready`, `explore_status`, `stopping`, `escalate`, `stopped`, `outcome`, `retried`, `retries_exhausted`, `calibration`, `resources`, `host_event`, `processed`, `recording_deleted`, `leftover_gone`, `message`, `error`, `manager_finished` |
| `resources.jsonl` | manager | 1 Hz | machine CPU, memory and disk; each run's CPU, memory, RTF and simulation time; concurrency and target |
| `control.jsonl` | CLI and GUI | on request | `pause`, `resume`, `cancel` |
| `status.jsonl` | run agent | 1 Hz, and events | ticks (simulation time, RTF, readiness, explore status, map area, true position, launch state, recording size); events (`agent_started`, `launch_started`, `explore_status`, `stop_requested`, `launch_shutdown`, `process_died`, `launch_still_running`, `recording_failed`, `agent_error`); a `final` record |
| `live.json` | manager | 2 s | the run screen's state: active runs and recent messages (BNR5) |
| `experiment.json` | manager and CLI | on change | state history, execution settings, provenance, planned runs, summary |

JSON and YAML writes are atomic: a temporary file in the same folder, `fsync`, then
`os.replace` (DR7). `fileio.check_schema` refuses a newer version and names the one it needs.
**Why:** DNR1 needs data written before a crash to stay readable.

### Pause, cancel and shared memory (BR15, BR16)

Pause sets `paused`, and no new run starts. Cancel stops every active run as `interrupted`
(reason `cancelled`), and those runs are not repeated. The results are still built, and the
experiment ends as `cancelled`. Each finished run runs `fastdds shm clean`, so a killed process's
shared memory in `/dev/shm` does not fill WSL's memory.

## Module C: resources and load

### Measuring the machine (CR1)

| Quantity | Source | Unknown when |
|----------|--------|--------------|
| Logical CPUs | `os.sched_getaffinity` | not in practice |
| Memory total and available | `/proc/meminfo` (`MemAvailable`) | the file is missing |
| Free disk | `shutil.disk_usage` on the data directory's nearest existing parent | the path cannot be read |
| Renderer | the `Renderer:` line of `tools/check_gpu.py`; "llvmpipe" or "softpipe" means software | the script fails |
| GPU memory | `nvidia-smi`, on the PATH or in `/usr/lib/wsl/lib` | no vendor tool |
| systemd user scopes | probe: `systemd-run --user --scope --quiet true` | the probe fails |

Anything unmeasured is `None`, shown as "unknown". **Why:** a GPU PC and a CPU-rendering PC differ
by an order of magnitude in run cost (2.2, item 5), so the suite measures rather than assumes (S1).

### Allocation and calibration (CR2, CR3)

The allocation is the CPUs, memory and disk the experiment may use. The defaults are half the
logical CPUs (at least one), half the available memory and half the free disk. It is stored in
`experiment.json`, not in the configuration (1.4).

When the calibration run ends, `Manager.calibrate` writes `calibration/<machine key>-<profile>.json`:
average and peak CPU cores, peak memory, disk bytes per simulation minute, RTF mean and minimum,
durations, and the measurement time. It counts only if the run was ready, has CPU samples, and reached
60 s of simulation time. The machine key hashes the CPU model, logical CPUs, memory (rounded to GiB)
and renderer. A calibration is reused for seven days, and `--recalibrate` ignores it.

### Sampling (CR8)

Each tick samples the machine from `/proc/stat`, `/proc/meminfo` and the disk. `RunSampler` samples a
run from `cpu.stat` and `memory.current` when its leader is in a systemd `.scope`, otherwise by summing
`/proc/<pid>/stat` over the group, listing the members every 2 s. CPU cores are the change in CPU
seconds over the change in wall time, and peak memory is kept. The cost against CNR2 is not measured yet.

### Concurrency cap (CR4)

`resources.concurrency_cap` returns the smallest of:

| Limit | Formula |
|-------|---------|
| Free domain IDs | free IDs plus active runs (`leases.free_count`) |
| User maximum | `max_concurrency`, or 64 when none is given |
| CPU | (allocated CPUs, minus one when there are three or more) ÷ average CPU cores of one run. One CPU stays free for processing (ER4). |
| Memory | allocated memory ÷ (peak memory of one run × 1.25) |

Without a calibration, the cap is 1 (CR3).

### The concurrency controller (CR4, CR5)

`ConcurrencyController` holds a target, which starts at 1. A new run may start only while fewer runs
are in progress than the target, and none is held back. Each tick:

*   **Lowering.** A run's RTF has been below the minimum for 30 s (`LOW_RTF_SECONDS`), the target is
    above 1, and the last change was at least 30 s ago: the target drops by one. The failed level is
    not tried again for 10 minutes, then 20 after a second failure; after two failures it is not
    tried again in this experiment (the cap drops below it).
*   **Raising.** The target is below the cap, no run is below the minimum, every active run is ready
    and at or above the minimum now, the last change was at least 60 s ago (`RAISE_SETTLE_SECONDS`),
    and the remembered ceiling does not block it: the target rises by one.
*   **Holding back.** While any run is below the minimum, no run starts. This is immediate.

A single run that cannot keep the minimum (CNR3) produces one warning, and then runs alone, flagged
`low_rtf`. **Why:** Nav2 misses its control deadlines when the simulation runs slowly, so the robot
behaves differently (CR5). Raising one run at a time, with a settle time, finds the highest level
that holds without swinging. **Why the backoff:** on the development PC, two runs at once dropped
both to an RTF of about 0.45. Each new try at that level slowed two runs for many minutes, so
trying forever cost more than it could gain.

### Confinement (CR6)

`resources.launch_prefix` wraps each agent. With systemd user scopes, and a memory limit or cores to
set, it uses `systemd-run --user --scope --quiet --collect -p MemoryMax=<bytes>`, plus
`-p AllowedCPUs=<list>` when the cpuset controller is delegated. Otherwise the cores come from
`taskset -c <list>`, the fallback CR6 asks for. `nice -n 10` is always added. Each run gets an equal,
disjoint share of the allocated CPUs, by slot; the allocated CPUs are the highest-numbered, since CPU 0
usually carries the desktop and interrupts. The memory limit is `max(allocated memory ÷ cap, 1.5 × peak)`
once a calibration exists. Without systemd, the manager checks each run's memory itself (rule 7).

**Why:** the suite must not need root (CNR1, S3), and user scopes and `taskset` need neither.

### Protecting the machine (CR7)

Each tick, in order: below 5% available memory, the newest running run is stopped as `infra_failure`
(`low_memory`) and repeated; below 10%, no new run starts; and if the calibration predicts more disk
use than the allocation or the free space allows (per run, times the queued and active runs), the
experiment pauses with reason `disk`, and only the user resumes it.

### Estimates (CR9)

`resources.estimate` uses the calibration. Runs are M × R over all sets. Disk per run is the
calibrated bytes per simulation minute times the calibrated minutes. `metrics_only` multiplies the rate
by 0.3, a default that is not measured. Retention changes the total: `all` keeps every run's disk,
`failures` assumes half, and `none` keeps only the runs at once. Duration is the calibration run's wall
time × runs ÷ concurrency. Durations are not scaled by maze size: on the development PC, runs on
2 × 2 m and 5 × 5 m mazes took about the same time (explore_lite spends most of a run on stalls and
the return), so scaling by area overestimated six times.

## Module A: maze generation

Module A is pure Python, with no ROS or Gazebo (ANR1), so its tests run in CI.

### Parameters and validation (AR5 to AR7)

The cell pitch is the corridor width plus the wall thickness. The grid is `round(width ÷ pitch)` by
`round(length ÷ pitch)` cells. Dimensions are between outer wall centre lines (AR5). `validate` returns
every error and warning. The pitch must divide both dimensions within 1 mm, and the message gives the
nearest valid corridor widths (AR6). Walls are at least 0.01 m thick (AR7). With a profile, corridors must
be at least 2 × navigation radius + 2 × costmap resolution (0.32 m for the Burger, an error), narrower than
the tested width gives a warning (0.48 m), and walls must be at least the lidar mount height + 0.05 m. A grid
has at most 100 cells per side.

### Carving, loops and the start (AR8, AR10, AR11)

`generator.generate` uses one `random.Random` seeded with the maze's seed:

1.  **Carve.** A randomized depth-first search from a random cell, with backtracking, removing the wall
    to a random unvisited neighbour. The result is a perfect maze: one path between any two cells.
2.  **Loops.** `loop_fraction` of the inner walls, chosen at random, are removed (AR11).
3.  **Start.** The configured cell, or with `random`, a random cell from the same generator (AR10).
4.  **Exits.** As below.

The spawn pose is the start cell's centre with heading 0. The launch file has no heading argument (AR10).

### Exits (AR3)

An exit is a one-cell gap in the outer wall. The outer wall is a cycle of boundary segments, numbered
clockwise from the top-left corner. Every pair of exits must satisfy two rules.

*   **Separation.** Their distance along the boundary, in cells, is at least `min_exit_separation`
    (default 2), counted on the cycle.
*   **No row with exits on both sides.** Never the left and right exit of one row. **Why:**
    `make_maze.py` drops blank lines of the layout. A row with no wall in its line would vanish, and the
    walls would no longer match (AR2).

The start cell is excluded as an exit location unless `allow_exit_in_start_cell` is set (YAML only). The
picker tries 200 random orderings. If none gives enough exits, it takes the greedy packing from a random
first exit. `max_exits` tries every first exit and keeps the best greedy packing. Its comment argues this
is the exact maximum on the closed boundary. The tests check it on small grids and one 10 × 10 grid.

### Checks (AR4, AR12)

After the exits are placed, `generate_checked` requires that a breadth-first search from the start reaches
every cell and every exit, that every exit is a gap, that no layout line is blank, and that the maze is not
a duplicate of an earlier one in the set. Reachability is checked, not assumed from the algorithm (AR4). A
failing maze is discarded, the next seed is tried, and the discard is logged as `maze_discarded`. After 1000
seeds, generation fails.

### Seeds (AR8)

Maze *i* of a set uses `derive_seed(set seed, i)`: the first six bytes of the SHA-256 of
`maze-experiments:<set seed>:<i>`. It is the same on every machine and Python. Same parameters and seed
give byte-identical files, which the property test checks.

### The files (AR1, AR2, AR9, AR13, AR14)

Per maze, in `mazes/set-NN/maze-NNN/`:

*   **`layout.txt` (AR2).** The `maze_sim/layouts` format: cells three characters wide, `+--` a wall on a
    cell's top edge, `+  ` an opening, `|` a wall on a cell's left side. `#` lines record the seed, grid,
    start, exits, and a note when the geometry is not the 0.5 m one.
*   **`world.sdf` (AR1, AR9).** The plugins and world structure of `make_maze.py`, the same wall boxes (a
    copy of `make_maze.py`'s placement, applied to the layout), and a ground plane centred on the maze, sized
    to the maze plus lidar range + 0.5 m on each side (AR9). `make_maze.py` uses a fixed 10 m plane.
*   **`maze.json` (AR13).** Parameters, seeds, grid, start, spawn pose, exits (side, cell, gap centre, boundary
    index, shortest path), walls, free area, ground plane, discarded seeds. It is Module E's ground truth.
*   **`preview.png` (AR14).** A small PNG from a short encoder in `files.py`, with the monitor's colours.

**AR2 note.** `make_maze.py` fixes `CELL = 0.5`, `THICK = 0.02` and `HEIGHT = 0.3` at module level, so a
layout reproduces identical walls only for that geometry. Other geometries get the correct `world.sdf`, and
their layout comment says so. The working tree of `maze_sim/scripts/make_maze.py` also adds a layout argument
and `-o` (not committed), which is what makes `make_maze.py <layout>` work.

## Module D: data and provenance

*   **Self-contained (DR4).** Paths inside an experiment are relative, and the configuration is copied in.
    `import_folder` recreates a missing configuration from that copy.
*   **Run record (DR1).** `runs/<id>/run.json` is the monitor's run format, written by `RunStore`. The suite's
    fields are in its `experiment` block: IDs, key, attempt, maze path, domain, partition, outcome, reason,
    `counts_as_robot_result`, recording state, RTF, resources and the calibration flag.
*   **Recording rules (DR1, DR2).** `recording.monitor_config` loads the monitor's `monitor.yaml` and adds the
    profile's exclusions. `metrics_only` adds `/scan` and `/imu`.
*   **Finished (DR8).** A run is finished when it has an outcome, a `metrics.json`, and a closed recording
    (`metadata.yaml`, or deleted by retention). Anything else is an interrupted attempt (BR12).
*   **Retention (DR9).** `apply_retention` keeps everything (`all`), deletes the recordings of mission successes
    (`failures`), or deletes all (`none`). The run record then says `recording: deleted`. Metrics stay.
*   **Provenance (DR6).** `provenance.collect` runs at creation. It records the capstone commit, branch, uncommitted
    changes (the changed files and a SHA-256 of `git diff HEAD`); each repository pinned in `capstone.repos`
    (pinned version, checked-out commit, local changes, diff hash); the SHA-256 of each patch and parameter file;
    the robot profile; the ROS, rclpy, Nav2, slam_toolbox and Gazebo versions; and the machine (CPU, logical CPUs,
    WSL memory, renderer, GPU memory, kernel, and a hash of the host name). The results warn on uncommitted changes.
*   **Listing (DNR2).** `ExperimentStore.list` reads only `experiment.json` files.

## Module E: results

### One pass over the recording (ER1, ENR1)

`metrics.run_metrics.extract` reads a run's bag once. Each message goes to a maze_monitor `Session` in replay
mode, which builds the analyzers' events and series exactly as `maze_monitor --replay` does. The same loop keeps
the messages the extractors need: `/clock`, `/ground_truth/poses`, `/odom`, `/odometry/filtered`, the `map`→`odom`
and `odom`→`base_footprint` transforms, `/map`, `/explore/status`, `/explore/exits` and `/rosout`. `compute` is pure
numpy, so its tests need no ROS graph (ENR2). `process_run` writes `metrics.json` with `pipeline_version`. The timing
target in ENR1 is not measured yet.

### Frame and metrics

The truth is `/ground_truth/poses`. The origin is the true pose at the first ground-truth sample, and every
estimate is compared with the truth relative to that origin. **Why:** the truth and the estimates must share a
frame, and the start pose is the one in which the map and odometry mean something.

| Group | Metric | Definition |
|-------|--------|-----------|
| Time | `exploration_end_s`, `mission_s` | Simulation time of the first explore-end status, and of the last return status (or the end of the run). |
| Time | `wall_s`, `rtf_mean`, `rtf_min`, `low_rtf` | From the run record. |
| Exploration | `distance_m`, `coverage`, `time_to_coverage_50/90/100_s` | Path length from ground truth; coverage as below. |
| Exits | `found`, `missed`, `false`, `matches` | One-to-one matching as below. |
| Mapping | `wall_precision`, `wall_recall`, `occupied_cells` | As below. |
| Localization | final and maximum position and heading error for `wheels`, `ekf`, `slam`; `jumps` | As below. `jumps` counts the monitor's "Localization jump" events. |
| Return | `distance_to_start_m`, `returned` | Last true position to the start, and whether `returned_to_origin` was seen. |
| Safety | `min_clearance_m` | Least distance from a true pose to a wall box, minus the footprint radius. |
| Safety | `contact`, `near_wall`, `blocked`, `stuck`, `spinning`, `oscillation`, `wheel_slip`, `bump`, `collision` | Counts of the monitor's events with those keys and categories. |
| Navigation | goals sent, succeeded, failed; recoveries, replans, costmap clears | The monitor's `new_goal`, `nav.*` and `clear_*` events and values. |
| Health | `warnings`, `errors`, `by_node` | `/rosout` at WARN (30) and ERROR (40) or above, by node. |
| Outcome | `mission_success` | Outcome `complete`, every true exit found, no false exit (ER2). `null` for non-robot outcomes. |

Sampling caps: 2000 ground-truth samples for localization, 4000 for clearance, and 200 points in the series
the charts use.

### Coverage, exits, mapping and localization (ER1)

**Coverage.** The free floor is sampled every 5 cm, keeping 5 cm from every wall box (`free_samples`). Each
map is scored by the share of those points whose cell is known (not `-1`). "100%" means 99.5% or more
(`COVERAGE_LEVELS`). **Why:** a single unseen cell at a corridor's edge should not count as a failure to explore.

**Exits.** The detected exits are the centroids in the last `/explore/exits` message, moved into the world frame.
Matching is one-to-one: pairs within the tolerance are sorted by distance, and each true exit and each detection is
used once. The tolerance is `analysis.exit_match_tolerance_m`, defaulting to one cell pitch. A missed exit is
"next to start" when its cell is at Manhattan distance 1 or less from the start cell.

**Mapping.** Occupied cells have a value of at least 65. Precision is the share of occupied cell centres within one
map resolution of a wall box. Recall is the share of points on the true walls' centre lines (every half cell) that
have an occupied cell within one cell plus half the wall thickness. The recall tolerance is wider, because a centre
line is half a wall thickness from the surface.

**Localization.** Three estimates are compared with the truth: wheel odometry (`/odom`), the EKF (`/odometry/filtered`),
and SLAM (the `map`→`odom` transform composed with `odom`→`base_footprint`). Each is interpolated at the ground-truth
times. The position error is the distance between positions, and the heading error is the wrapped yaw difference. The
specification says this follows `tools/drift_report.py`. The frame composition and per-estimate errors match that
script's structure, as far as a read of both shows.

### Rates, low RTF and findings (ER2 to ER7)

Rates are shares with 95% Wilson score intervals. Numeric metrics are described by count, mean, sample standard
deviation, median, 10th and 90th percentiles, minimum and maximum. `final_runs` takes each planned run's last
robot-result attempt. Excluded attempts are kept in their own list and reported by outcome, never dropped (ER3).
Rates use processed runs as the denominator.

`rtf_effect` compares the mission success of runs with `low_rtf` and the rest with a two-sided Fisher exact test.
A p value below 0.05 adds a warning that machine load may have affected the results (ER6). **Why:** CR5 keeps RTF high
on purpose, and ER6 checks whether that was enough for this experiment.

`metrics.findings` writes the plain sentences and tables for each category (Overview, Outcomes, Exploration, Exits,
Mapping, Localization, Safety, Performance), for the whole experiment and each set. The GUI, the Markdown export and the
text report share this code. `metrics.fmt` gives durations as m:ss (h:mm:ss beyond an hour), percentages to one decimal,
lengths below one metre in centimetres, and angles in degrees (ER7).

### Results files and export (ER5, ER8, ER9, ER10)

`build_results` writes `results/` in one pass: `results.json` (aggregates, findings, chart list, one row per run, warnings;
the GUI reads it), `runs.csv` (one row per attempt), `report.md` (the export without raw messages), and `charts/*.png`.
Results come only from the recordings, the maze files and the status files. `reprocess` rebuilds them, and the pipeline
version is written with every result (ER5).

The Markdown export has a description of the suite and of the mission, the summary, the configuration and execution
settings, the provenance, the tables per set, one row per attempt, and the failed runs with their reasons and first six
events. It can add one run's raw messages (`--raw-run`). It must stay below 200 KB: if it is larger, the events per failure
are halved until it fits, and the raw section counts toward the size (ER8). CSV and JSON carry the same results, and the
JSON also has each run's metrics and the configuration.

`metrics.echo` writes a message as `ros2 topic echo` does, summarizing arrays of more than 16 numbers (ER9). Charts use
matplotlib with the Agg backend and the monitor's colours: `outcomes.png` (per set), `distributions.png` (box plots of the
main metrics per set), `coverage.png` (coverage over time, a line per run), and `drift.png` (position error over time).
A chart that fails is recorded with its error, and the rest of the results are still built (ER10).

## Module G: the user interface

The application is one `QMainWindow` with a stack of screens (`gui/app.py`): the home screen (two panels, search, sort,
menus; GR2, GR3), the configuration editor (GR5), the run dialog (CR1, CR2, CR9), the run screen (GR6) and the results
screen (GR7). The GUI reads and writes files only through `Suite`, and never runs an experiment itself.

**Background work (GNR1).** `gui/common.background` runs a function on a `QThread`, with a `QObject` worker. Results and
failures, with tracebacks, return to the GUI thread by signal. A progress dialog appears when a text is given. Reading
recordings, measuring the machine, checking configurations, exporting, importing and loading results all go through it.
**Why:** a 100-run experiment's recordings are large, and reading them on the GUI thread would freeze the window.

**Refresh.** The home screen reloads every 5 s while it is visible. The run screen takes a snapshot every second, and does not
start another while one is running. Results load once, and are rebuilt on request, or when they are older than the run metrics
and no manager is running.

**Style (GNR3).** `gui/common.py` imports `style` from `maze_monitor.gui`, so the same colour means the same thing in both tools.

**Desktop (GR4).** Open folder uses Nautilus, or Windows Explorer through `wslpath -w`. Open terminal here uses Windows Terminal
(`wt.exe`), or a Linux terminal. When neither exists, the path is copied and the status bar says so.

**Editor (GR5).** Fields are checked as the user types, with `configuration.check` (without the workspace and data directory
checks). The corridor width list comes from `valid_corridor_widths`, up to 3 m. The sample maze is maze 000. Save is disabled while
there are errors.

**Run screen (BNR5).** It reads `live.json`, `resources.jsonl` and the run folders, so reopening it shows the same state. When the
experiment completes while the screen is open, the results open (FR7).

**Results screen (GNR2).** It reads `results/results.json`, so a 100-run experiment opens without reading every recording. The raw
data tab reads a run's bag only when the run is expanded, and then only the selected topic, 200 messages a page, through a
`StorageFilter`.

## Testing

The unit tests in `test/` run without a ROS graph (S4):

| Test | What it checks |
|------|----------------|
| `test_property_1000_mazes` | ANR3: over 1000 mazes from fixed-seed random valid parameters: AR3 (exit count, separation, start, exits as the only gaps), AR4 (reachability), AR6 and AR7, AR8 (byte-identical regeneration), AR9 (ground plane and walls). |
| `test_layout_matches_make_maze` | AR2: for 20 seeds of the 0.5 m geometry, `make_maze.py`'s `build` gives the same walls as the layout. |
| `test_existing_layouts_parse_like_make_maze` | The wall parser agrees with `make_maze.py` on the repository's own maze. |
| `test_speed_20x20` | ANR2: a 20 × 20 maze with its checks takes under 1 s. |
| `test_set_is_reproducible_and_distinct` | AR8 and AR12: a set regenerates identically, and its mazes differ. |
| `test_validation_rules` | AR3, AR6, AR7: the pitch rule with suggestions, the corridor minimum, the tested-width warning, wall thickness and height, and the exit maximum. |
| `test_metadata` | AR13: `maze.json` holds parameters, walls, exits with world coordinates, the spawn pose and shortest paths. |

`test_metrics.py` covers Module E with synthetic data (ENR2): a perfect run is a mission
success with full coverage, exact wall precision and recall, the expected localization errors
and clearance; a missed or false exit fails the mission; a shifted map loses precision; exit
matching is one-to-one; the Wilson interval and Fisher's exact test against known values;
excluded runs are counted separately; the low-RTF warning (ER6); value formatting (ER7); array
summaries (ER9).

`test_suite.py` covers Modules B, C, D and F: atomic JSON with `schema_version` and clear errors
for newer files (DR5, DR7); append-only `.jsonl` that survives a torn last line (DNR1); IDs;
refusing a data directory under `/mnt/`; sweeps expanding into every combination (FR3); every
configuration problem reported at once (FR4); immutable configurations with copied parameter
files, "Create modified version" and the delete guard (2.3.3, FR8); the round-robin, deterministic
run order (BR14, FNR1); the concurrency cap and the controller's raise and lower rules (CR4, CR5);
disjoint CPU shares (CR6); every outcome rule (BR3, BR5, BR6, BR7, BR9), including deaths during
the launch's own shutdown; the queue's retry counting (BR10) and recovery of active attempts
(BR12); importing an experiment whose configuration is missing (DR4); and listing 1000
experiments in under 2 s (DNR2).

CI runs all three files after the build (`.github/workflows/build.yml`). The isolation test
(BNR2) and the other acceptance tests of 5.0 need the simulation, so they were run by hand; their
results are in the user manual.

## Limits and known deviations

*   **Recordings deleted by the retention policy.** Reprocessing skips such runs and keeps their
    existing metrics, so a metric added later can't be computed for them.
*   **Clock-jump action and retry counting.** The `flag` action is an opt-in departure from BR13 (Module B). A clock jump's
    `infra_failure` uses the retry budget. BR10 reads that way, but the specification does not say so.
*   **Estimates are rough.** The `failures` factor of 0.5, the `metrics_only` ratio of 0.3 and the duration from one calibration
    run are assumptions, not measurements.
*   **Editor gap.** `allow_exit_in_start_cell` is YAML only.
*   **AR2 geometry.** `make_maze.py` hard-codes 0.5 m cells with 0.02 m x 0.3 m walls, and it
    must not be changed. A generated layout therefore reproduces identical walls through
    `make_maze.py` only for that geometry; for other sizes the layout says so in a comment and
    `world.sdf` is the reference. The `make_maze.py <layout>` command line itself is an
    uncommitted change in the developer's working copy; the test calls its `build()` function,
    which exists on `main`.
*   **Exits on both sides of one row.** Exits are never placed on the left and right side of the
    same row, because `make_maze.py` drops a layout line with no wall at all. This is a small
    restriction on top of AR3.
*   **Measured targets.** See the user manual's acceptance results for ENR1, BNR3, CNR2, GNR2 and
    DNR2 as measured on the development PC.
*   **Heading and platform.** The spawn heading is always 0 (AR10). The manager and run finder read `/proc` and use `flock`
    and `systemd-run`, so the suite runs only on Linux (WSL).
*   **Acceptance tests.** Run on the development PC; results in the user manual. That PC's WSL
    clock was stepped back about 0.9 s every 31 s throughout, so the tests ran with
    `clock_jump_action: flag`, and the robot results they produced are not representative.
