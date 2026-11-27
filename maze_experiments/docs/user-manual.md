# Maze experiments user manual

The maze experiments suite runs the simulated robot on many randomly generated
mazes, records every run, and turns the recordings into results. Use it to find
out how often the robot's software (SLAM, Nav2 and explore_lite) explores a maze,
finds every exit and returns to the start, and what goes wrong when it does not.

This manual explains how to start the suite, create a configuration, run an
experiment, read the results, share them, and use the command line. For how the
suite works inside, see the [design document](design.md).

## What the suite is for

A configuration describes one or more *maze sets*. Each set has its own maze
size, corridor width, number of exits, number of mazes and repeats. An experiment
runs one configuration. It generates every maze first, runs the robot on each
one, records each run in the maze monitor's format, and computes the metrics of
every run. It then summarizes them per maze set and for the whole experiment.

The mission is: explore the whole maze, identify every exit, then return to the
start. A run is a **mission success** when it completes that mission, finds every
true exit and reports no false exit.

The suite measures the software stack in simulation. It does not measure a
physical robot, and it never changes the stack's settings. The simulated robot is
a TurtleBot3 Burger, standing in for the four-wheel robot until a model of that
robot exists. The results describe the software, not the drivetrain.

## Before you begin

*   Build the workspace with `./setup.sh`, as the repository `README.md` describes,
    and open a new terminal afterwards. The suite is built in simulation mode.
*   The host is a Windows PC with WSL 2 and WSLg. The guest is Ubuntu 24.04 with
    ROS 2 Jazzy and Gazebo Harmonic.
*   Data is kept in `~/maze_experiments` by default. It must be on the Linux file
    system. The suite refuses a data directory under `/mnt/`, because SQLite on the
    Windows drive is slow and its file locking is unreliable.
*   The suite needs no `sudo`. It works without a GPU, but then every run is slower.

## Start the suite

```bash
ros2 run maze_experiments maze_experiments     # or: maze_experiments
```

With no arguments the command opens the application at the home screen. With a
subcommand it runs the [command line](#command-line) instead.

The toolbar has **Back to Home** (Alt+Left), **Refresh** (the home screen also
reloads every 5 seconds), **New configuration**, **Import**, **Machine** (what
this PC gives the suite) and **Manual** (this manual, inside the application).

The data directory is chosen in this order: the `--data-dir` option, the
`MAZE_EXPERIMENTS_DIR` environment variable, `data_dir` in
`~/.config/maze_experiments/settings.yaml`, and finally `~/maze_experiments`.

## The home screen

The home screen has two panels:

*   **Configurations** (left): each configuration with its number of runs, maze
    sets, experiments and creation time. A configuration that has never been run
    says "not run yet", in gray.
*   **Experiments** (right): each experiment, newest first, with its state, date,
    configuration, runs done and mission success rate.

Both panels have a **search** box, which matches any text in a row, and a **sort**
list: Newest first, Oldest first, Name A–Z, Name Z–A.

Double-click a configuration to view it, read-only. Double-click an experiment to
open it: the run screen while its manager runs, otherwise the results.

| Right-click on | Menu items |
|----------------|------------|
| Configuration | Run this configuration, Create modified version, View, Delete, Open folder, Open terminal here, Copy path |
| Experiment | Open results, Show run screen (only while running), Resume (only when interrupted, or paused with no manager), Delete, Export…, Open folder, Open terminal here, Copy path |

A configuration can be deleted only when no experiment uses it. An experiment's
deletion removes its runs and recordings, after a confirmation. A running
experiment cannot be deleted; cancel it first. An experiment that looks active
but has no live manager is shown with "(no manager)".

## Create a configuration

Click **New configuration**. The editor has three columns (General, Maze sets, and
the sample maze with the totals) and a problem list at the bottom.

### General

| Field | Meaning |
|-------|---------|
| Name | Required. Shown in the lists. |
| Robot profile | The robot to simulate. See [Robot profiles](#robot-profiles). |
| Run limits | Timeouts, stall rule, retries and minimum real-time factor. The defaults are in [Configuration file format](#configuration-file-format). |
| Recording profile | `full` records what the maze monitor records by default. `metrics_only` also drops `/scan` and `/imu`, which makes recordings about a third smaller. The lidar-based safety events are lost with it. |
| Keep recordings | `all` (default), `failures` (keep only the recordings of runs that were not mission successes), or `none`. Metrics are always kept. |
| Exit match tolerance | Blank means one cell pitch. A detected exit counts as a true one within this distance of the true exit's gap centre. |

**Why:** the configuration sets the recording rules, so every run in an experiment
is kept or removed by the same rule.

### Maze sets

Use **Add**, **Duplicate** and **Remove** to manage the sets. Each set has:

| Field | Meaning |
|-------|---------|
| Width, Length | Size in metres, between the centre lines of the outer walls. |
| Corridor width | Clear width between two wall faces, in metres. |
| Wall thickness | In metres. At least 0.01 m. |
| Wall height | In metres. At least the lidar mount height plus 0.05 m (0.221 m for the Burger). |
| Exits | Number of one-cell gaps in the outer wall. |
| Min. exit separation | Smallest distance between two exits, in cells, along the outer wall. |
| Loop fraction | Share of inner walls removed after generation. 0 gives a perfect maze. |
| Start, Start cell | `fixed` uses the start cell given; `random` picks a different one in each maze. Row 0 is the top row. |
| Mazes (M), Repeats (R) | The set has M × R runs. |
| Seed | The same seed gives the same mazes. |
| Stack parameter files | Optional Nav2, SLAM, explore_lite and EKF parameter files. Blank uses the robot profile's files. The chosen files are copied into the configuration. |

**Sweeps.** A comma-separated list in a maze field sweeps that parameter.
`exits: 1, 2, 3, 4` gives four sets. Lists in several fields give every
combination. Each generated set's name shows its swept values, for example
`5x5-2exits (exits=2)`.

### Valid corridor widths

The cell pitch is the corridor width plus the wall thickness. It must divide the
width and the length exactly (to within 1 mm). The **Valid corridor widths** list
shows the widths that fit and the grid each one gives. Choose one to fill in the
corridor width. If the value you typed does not fit, the list says so, and the
problem list names the nearest valid widths.

The editor also checks the robot profile. Corridors below the profile's minimum
are errors: 0.32 m for the Burger (2 × navigation radius 0.11 m + 2 × costmap
resolution 0.05 m). Corridors below the width the stack was tuned on (0.48 m for
the Burger) are warnings, because results may then reflect a maze the stack was
not tuned for.

### Sample maze and totals

The right column draws maze 000 of the selected set, generated with the set's
seed: walls black, exits green, start blue. The totals give the sets after the
sweeps and the number of runs. With a calibration for this PC and profile, they
also estimate duration and disk use at the default allocation. Otherwise they say
the estimate comes after the first run.

### Saving

**Save** is enabled when there are no errors and the configuration has a name.
Warnings do not stop a save. Saving creates a new configuration. It never changes
one. The problem list reports every problem at once, for example:

*   `The cell pitch 0.500 m (corridor 0.480 + wall 0.020) does not divide the width 5.100 m. Nearest valid corridor widths: ...`
*   `2 exits do not fit a 4 x 4 grid with exits at least 2 cells apart: at most 1.`

### Change a configuration

A saved configuration cannot be changed, so an experiment always matches the
configuration it used. To change one, right-click it and choose **Create modified
version**. The editor opens with every field copied and "(modified)" added to the
name. Change what you need and save. The original stays as it was. On the command
line, copy the YAML file, change it and run `config create`.

## Run an experiment

### The run dialog

Right-click a configuration and choose **Run this configuration**. The dialog
measures this PC, which takes a few seconds, and shows:

*   **This machine:** CPU model, logical CPUs and memory available to WSL, free
    disk in the data directory, the renderer Gazebo will use (GPU, or software
    rendering on the CPU), and GPU memory if a vendor tool reports it.
*   **Allocation:** CPUs, memory and disk the experiment may use, and the most runs
    at once. The defaults are half of the CPUs, half of the available memory and
    half of the free disk.
*   **Estimate**, from the calibration if there is one, and any warnings or errors.
    Start is disabled while there is an error.
*   Two check boxes: **Measure a new calibration**, and **Only flag runs during a
    WSL clock jump instead of repeating them** (see [WSL clock jumps](#wsl-clock-jumps)).

**Why:** the suite does not assume how much a PC can do. It measures the machine,
and you decide how much of it the experiment may use.

### What happens when you start

1.  The suite checks the configuration again and copies it into the experiment
    folder. It records the provenance: the code commit and whether it had
    uncommitted changes, the pinned versions of the other repositories, a hash of
    every parameter file, the ROS and Gazebo versions, and the machine.
2.  A background manager starts. The run screen opens.
3.  **Generating.** Every maze is generated before the first run, so the mazes are
    fixed. Mazes that fail a check are logged as discarded in `manager.jsonl`.
4.  **Calibrating.** The first run runs alone. The suite measures its CPU use,
    memory, disk written per minute of simulation time, and real-time factor. The
    calibration is saved for this PC and robot profile, and reused for seven days.
    A calibration run counts only if it reaches 60 s of simulation time.
    Otherwise the next run calibrates.
5.  **Running.** Runs start at the concurrency the machine allows (below). Each run
    is recorded and processed as soon as it ends, so results grow during the
    experiment.
6.  **Completed.** When the last run is done, the results are built. If the run
    screen is open, the results open.

### Concurrency

The suite starts with one run, even when a calibration exists. It raises the number
by one when every active run is ready, at or above the minimum real-time factor
(RTF, default 0.8), and the last change was at least a minute ago. It lowers the
number by one when a run stays below the minimum for 30 seconds. While any run is
below the minimum, no new run starts. After a lowering, the suite does not return
to that level for 10 minutes.

The limit is the smallest of: the allocated CPUs divided by the average CPU use of
one run; the allocated memory divided by one run's peak memory plus 25%; the free
ROS domain IDs (1 to 101); and the **At most at once** value. With no value set, the
suite never runs more than 64 at once.

**Why:** under load the simulation runs slowly, Nav2 misses its control deadlines,
and the robot behaves differently. Results from such runs would measure the load,
not the software. The minimum RTF keeps runs close to real time, and the results
report separately how low-RTF runs fared.

### The run screen

*   The header gives the state, runs done, excluded attempts and mission success so far.
*   The concurrency line gives the runs at once, the target, the limit, whether runs
    are held back, and the queue.
*   The table lists every planned run with its state (queued, starting, running,
    stopping, processing, done, excluded), simulation time, RTF (amber below the
    minimum) and outcome. A repeated run shows its attempt number.
*   The graphs show the last 15 minutes: machine CPU and memory, runs at once against
    the target, and the RTF of up to eight runs with the minimum drawn in amber.

| Button | Action |
|--------|--------|
| Pause | Start no new runs. Active runs finish. |
| Resume | Continue a paused experiment. |
| Cancel | Stop the active runs and end the experiment. Finished runs are kept. The suite asks first. |
| Results so far | Open the results of the runs that have finished. |

Closing the window does not stop the experiment. The manager is a separate
background process. Double-click the experiment, or choose **Show run screen**, to
see it again. If the disk would run out, the experiment pauses with a message.
Free some space or raise the allocation, then resume.

### Pause, resume and cancel from the command line

```bash
maze_experiments pause <experiment>
maze_experiments resume <experiment>
maze_experiments cancel <experiment>
```

Pause and cancel need a running manager. Resume works on a paused or an
interrupted experiment.

### Resume after a crash

If the manager, the GUI, WSL or the PC stopped, the experiment shows as interrupted,
or as active with "(no manager)". Right-click it and choose **Resume**, or run
`maze_experiments resume <experiment>`. The suite then:

*   stops the processes left over from the runs that were active, checking each
    process's start time, so it never stops an unrelated process that reused an ID;
*   marks those runs interrupted and queues them again as new attempts;
*   keeps every finished run and continues with the rest. No run is duplicated.

Interrupted runs do not use up retries (see [Outcomes and retries](#outcomes-and-retries)).

Killing the manager does not stop its runs at once. They keep going until they end,
and the suite stops them at the next start. Most commands, and the GUI at its start,
find experiments that were running without a manager, mark them interrupted, and
say so.

## The results

Double-click a finished experiment, or choose **Open results**. The results come
from files the suite wrote during the run, so a 100-run experiment opens quickly.
The top of the screen gives the state, the robot results, the excluded attempts and
the generation time. Warnings appear in amber. For example, the robot code had
uncommitted changes, or low-RTF runs succeed less often.

| Button | Action |
|--------|--------|
| Export… | Write Markdown, CSV and JSON files (see [Export](#export)). |
| Update results | Rebuild the results from the run metrics. A note appears when newer metrics exist. |
| Reprocess all runs | Recompute every run's metrics from its recording, then rebuild. This can take a while. Read [Rebuilding results](#rebuilding-results) first. |
| Open folder | Open the experiment folder. |

### Processed results

The **Show** list chooses the whole experiment or one maze set. The sidebar has
these categories, and **Runs** and **Provenance** at the end:

| Category | What it shows |
|----------|---------------|
| Overview | Mission success with its 95% interval, outcomes, exits found and false, median coverage and mission time. |
| Outcomes | How many runs ended in each outcome, with the share and its interval. Excluded attempts are listed separately. |
| Exploration | Time to the end of exploration, mission time, distance driven, coverage over time, and the distance from the start at the end. |
| Exits | Exits found, missed and false; the share of runs that found every exit; missed exits in or next to the start cell. |
| Mapping | Wall precision and recall of the final map. |
| Localization | Position and heading error of SLAM, the EKF and wheel odometry against ground truth, and localization jumps. |
| Safety | The closest approach to a wall, and counts of contact, near-wall, blocked, stuck, spinning and oscillation events. |
| Performance | Wall time, RTF, goals, recoveries, replans, and warnings and errors from the logs. The low-RTF comparison is here. |

Each category has plain sentences, then tables. A metric table shows the count,
mean, standard deviation, median, 10th and 90th percentile, minimum and maximum.
The whole-experiment view also has charts: outcomes per set, the spread of the
main metrics per set, coverage over time, and localization error over time.

### What the numbers mean

| Term | Definition |
|------|------------|
| Mission success | Outcome `complete`, every true exit found, and no false exit. |
| Coverage | Share of the free floor that the final map knows, sampled every 5 cm and kept 5 cm from walls. A run counts as 100% at 99.5% or more, so one unseen cell is not a miss. |
| Exit found | A detected exit within the match tolerance (one cell pitch by default) of a true gap centre. Each true exit matches at most one detection. |
| Mission time | Simulation time until the final return status, or the end of the simulation. |
| Wall precision | Share of occupied map cells within one map cell of a real wall. |
| Wall recall | Share of points along the real walls that have an occupied map cell within about one map cell. |
| Position error | Distance between an estimate (wheel odometry, the EKF or SLAM) and the true pose, both relative to the start pose. |
| Localization jump | A SLAM correction of more than 10 cm, or 5°, in one step. |
| Minimum clearance | Closest distance from the true pose to a wall, minus the footprint radius (0.105 m for the Burger). |
| RTF | Simulation seconds per wall-clock second. |
| 95% interval | Wilson score interval for a share. A wide interval means few runs. |

Rates use only runs that count as robot results. Metric statistics use the runs
that were processed.

### Raw data

The **Raw data** tab shows the recorded messages, in the form of `ros2 topic echo`.
Expand a run in the tree on the left to see its topics. The middle column lists the
messages of the chosen topic, 200 to a page, and the right column shows the selected
message as selectable text. **Copy** copies any message. **Open in maze monitor**
replays the selected run. A run whose recording was removed shows "(no recording)".

### Export

**Export…** asks for a folder and writes three files named after the experiment's ID:

*   `<id>.md`: a report written to be pasted into an LLM. It has the summary, the
    configuration, the execution settings, the provenance, the tables per set, one
    row per run, and the failed runs with their main events. It stays below about
    200 KB, by listing fewer events per failure if needed.
*   `<id>.csv`: one row per attempt.
*   `<id>.json`: all results, every run's metrics and the configuration.

If a topic is selected on the Raw data tab, the export asks whether to add its first
20 messages to the Markdown file. The command line writes the same files with
`maze_experiments export`, and a report with `maze_experiments report`.

### Rebuilding results

The results come from the recordings and status files, and can be rebuilt at any
time. **Update results** rebuilds them from the run metrics. **Reprocess all runs**
recomputes the metrics from the recordings first.

Runs whose recordings were removed by the Keep recordings setting keep their existing metrics when you reprocess.

## Share results

An experiment folder is self-contained. It has the configuration, the mazes, the
recordings and the results, and only relative paths inside it. A copy is meant to
work on another PC or user account. The acceptance test for this is still to be run.

| Right-click item | What it does |
|------------------|--------------|
| Open folder | Opens Nautilus if installed, otherwise Windows Explorer at the folder. |
| Open terminal here | Opens Windows Terminal in this WSL distribution at the folder, else a Linux terminal if one is installed, else copies the path and says so. |
| Copy path | Copies the folder's path. |

To give an experiment to a teammate, copy the folder into their
`~/maze_experiments/experiments/`, or send it (or a zip made with
`maze_experiments export <experiment> --zip`) and have them choose **Import**, or
run `maze_experiments import <folder>`. Import refuses an experiment whose ID already
exists. If the experiment's configuration is missing, import recreates it from the
copy inside the experiment. A configuration folder can be imported the same way.

## Command line

The command line uses the same code and files as the application.

```bash
maze_experiments [--data-dir DIR] <command> [options]
```

`--data-dir` goes before the command.

| Command | Purpose |
|---------|---------|
| `config` | `example`, `check FILE`, `create FILE [--name NAME]`, `list`, `show CONFIG`, `delete CONFIG` |
| `run CONFIG` | Start an experiment. |
| `status [EXPERIMENT] [--all]` | List experiments, or show one in detail. |
| `pause`, `resume`, `cancel EXPERIMENT` | Control a running or interrupted experiment. |
| `report EXPERIMENT` | Print or save results as text, Markdown, CSV or JSON. |
| `reprocess EXPERIMENT [--results-only]` | Recompute metrics and rebuild results. |
| `import FOLDER`, `export EXPERIMENT`, `delete EXPERIMENT` | Move, export or delete experiments. |
| `replay EXPERIMENT RUN` | Open a run in the maze monitor. |
| `machine` | Show what this PC gives the suite. |

`CONFIG` is a configuration's ID, its name if unique, or its folder. `config check`
lists warnings and errors, and exits with status 1 when there are errors.
`config create --name` replaces the name. `config list` prints ID, name, runs and
experiments. `config show` prints the saved file and its folder.

`status` without arguments lists the active and interrupted experiments and the ten
most recent others. With an experiment it shows the state, the outcomes, the
concurrency, the runs in progress and recent events.

### run

```bash
maze_experiments run CONFIG [--cpus N] [--memory SIZE] [--disk SIZE]
                     [--max-concurrency N] [--name NAME] [--recalibrate]
                     [--clock-jumps {retry,flag}] [--foreground] [--yes] [--force]
```

| Option | Meaning |
|--------|---------|
| `--cpus N` | Logical CPUs to allocate. Default: half. |
| `--memory SIZE` | For example `8G` or `512M`. Default: half of the available memory. |
| `--disk SIZE` | For example `50G`. Default: half of the free disk. |
| `--max-concurrency N` | Never run more than N at once. |
| `--name NAME` | Experiment name. Default: the configuration's name. |
| `--recalibrate` | Measure a new calibration. |
| `--clock-jumps` | `retry` (default) repeats runs active during a WSL clock jump. `flag` only marks them. |
| `--foreground` | Run the manager in this terminal. Ctrl+C stops the runs and leaves the experiment interrupted. |
| `--yes`, `-y` | Do not ask "Start?". |
| `--force` | Start even when the check reports errors. Use it only for estimate problems. |

The command prints the same summary as the run dialog, creates the experiment, and
starts the manager in the background. It then prints the commands to follow it.

### report, reprocess, export, replay

| Command | Options and effect |
|---------|--------------------|
| `report` | `--format {text,md,csv,json}` (default text), `-o FILE`, `--raw-run RUN` with `md` only, `--raw-topics` (default `/explore/status`), `--raw-max N` (default 20). |
| `reprocess` | Recomputes every run from its recording. `--results-only` only rebuilds the results. |
| `export` | Writes `<id>.md`, `.csv` and `.json` into `-o DIR` (default: current folder). `--zip` zips the experiment folder instead. |
| `delete` | Asks first. `--yes` skips the question. |
| `replay` | Runs `ros2 run maze_monitor maze_monitor --no-ros --replay` on the run. `RUN` is an ID such as `s00-m000-r00-a1`. |

## Configuration file format

A configuration is YAML. `maze_experiments config example` prints a complete one,
also kept in `config/examples/use_case_5x5.yaml`. The suite writes `schema_version`,
`id` and `created` itself.

| Key | Default | Meaning |
|-----|---------|---------|
| `name` | `unnamed` | Name shown in the lists. |
| `description` | empty | Free text. |
| `robot_profile` | `burger` | Robot profile name. |
| `run_limits.sim_timeout_s` | 900 | Run timeout in simulation seconds (BR5). |
| `run_limits.startup_timeout_s` | 120 | Wall-clock seconds for a run to become ready (BR3). |
| `run_limits.stall_window_s` | 120 | Simulation seconds without progress before a stall (BR7). |
| `run_limits.stall_min_area_m2` | 0.05 | Map growth that counts as progress (BR7). |
| `run_limits.stall_min_move_m` | 0.25 | Movement that counts as progress, in m (BR7). |
| `run_limits.retries` | 2 | Repeats of non-robot outcomes (BR10). |
| `run_limits.min_rtf` | 0.8 | Minimum real-time factor (CR5). |
| `recording.profile` | `full` | `full` or `metrics_only` (DR2). |
| `recording.retention` | `all` | `all`, `failures` or `none` (DR9). |
| `analysis.exit_match_tolerance_m` | null | Null means one cell pitch (ER1). |
| `maze_sets[].name` | `set N` | Set name. |
| `maze_sets[].maze` | see below | Maze parameters (Module A). |
| `maze_sets[].mazes` | 10 | M, mazes in the set. |
| `maze_sets[].repeats` | 1 | R, repeats of each maze. |
| `maze_sets[].seed` | N | Set seed (AR8). |
| `maze_sets[].stack_params` | none | Optional `nav_params`, `slam_params`, `explore_params`, `ekf_params` files (FR8). |

Maze parameters, under `maze_sets[].maze`:

| Key | Default | Meaning |
|-----|---------|---------|
| `width`, `length` | 5.0 | Size between outer wall centre lines, in m. |
| `corridor_width` | 0.48 | Clear corridor width, in m. |
| `wall_thickness` | 0.02 | In m. |
| `wall_height` | 0.30 | In m. |
| `exits` | 2 | Number of exits. |
| `min_exit_separation` | 2 | Smallest exit distance along the outer wall, in cells. |
| `allow_exit_in_start_cell` | false | Not offered by the editor. |
| `start` | `fixed` | `fixed` or `random`. |
| `start_cell` | `[0, 0]` | Column and row of the start cell. |
| `loop_fraction` | 0.0 | Share of inner walls removed to make loops (AR11). |

*   A sweep is a list value, for example `exits: [1, 2, 3, 4]`. `start_cell` sweeps
    only as a list of cells, for example `[[0, 0], [4, 4]]`.
*   An unknown key under `maze` is an error. Other unknown keys are not used.
*   A stack parameter path is absolute, or relative to the YAML file. The file is
    copied into the configuration.

## Outcomes and retries

Every run ends with exactly one outcome.

| Outcome | Meaning | Robot result | Retried |
|---------|---------|:------------:|:-------:|
| `complete` | Exploration complete, then returned to the start. | yes | no |
| `incomplete` | The explorer gave up on some frontiers. | yes | no |
| `return_failed` | Exploration complete, return to the start failed. | yes | no |
| `timeout` | Run timeout reached, in simulation time. | yes | no |
| `stalled` | No map growth and no movement for the stall window. | yes | no |
| `stack_crash` | A robot software process died after the run was ready, or explore_lite ended without a final status. | yes | no |
| `startup_failed` | Not ready in time, or the launch ended before ready. | no | yes |
| `sim_frozen` | The wall-clock limit passed while the simulation clock had stopped. | no | yes |
| `infra_failure` | Out of memory, host suspend, clock jump, agent exit, or unreadable recording. | no | yes |
| `interrupted` | The manager, WSL or PC stopped, or the run was cancelled. | no | yes, not counted |

*   A robot result is never retried. Retrying it would hide the failure and raise
    the success rate (BR10).
*   A non-robot outcome is repeated up to `retries` times (default 2). A run has at
    most three attempts by default, each in its own folder.
*   Attempts interrupted by a manager stop, a cancel or a manager crash do not use
    up the retries. Infrastructure failures do.
*   The results count each run's final robot result. Excluded attempts are listed
    and never dropped silently. Mission success is a separate flag.

## Data directory

```
~/maze_experiments/
├── configurations/<config id>/       configuration.yaml, params/
├── experiments/<experiment id>/
│   ├── experiment.json               state, configuration, execution settings, provenance
│   ├── configuration.yaml, params/   copies used by the experiment
│   ├── manager.jsonl                 the manager's log
│   ├── resources.jsonl               machine and run usage, once per second
│   ├── live.json                     what the run screen shows
│   ├── control.jsonl                 pause, resume and cancel requests
│   ├── mazes/set-00/maze-000/        world.sdf, layout.txt, maze.json, preview.png
│   ├── runs/<run id>/                run.json, bag/, status.jsonl, launch.log, metrics.json
│   └── results/                      results.json, runs.csv, report.md, charts/
├── calibration/                      one file per PC and robot profile
└── robots/                           your own robot profiles (optional)
```

*   `status.jsonl` is written once per second by the run's agent. It is the run's log.
*   A run ID such as `s00-m000-r00-a1` gives the set, maze, repeat and attempt.
*   `bag/` is a standard ROS 2 recording (SQLite). `ros2 bag info` reads it.
    `run.json` has the maze monitor's run format, so `maze_monitor --replay` opens it.
*   IDs start with the time they were made and add a random part, so folders from
    different PCs never collide.
*   Every JSON and YAML file has a `schema_version`. A file written by a newer
    suite is refused, with the version it needs.
*   Do not edit files inside an experiment folder. The suite reads them.

## Robot profiles

A robot profile describes the simulated robot: its model, spawn, bridge config,
footprint and lidar, and the stack parameter files tuned for it. The package ships
`burger`, in `config/robots/burger.yaml`. Profiles in `~/maze_experiments/robots/`
are found too, and the file name is the profile name.

| Key | Meaning |
|-----|---------|
| `name` | Profile name. Required. |
| `footprint_radius` | Robot centre to outer edge, in m. Used for minimum clearance. Required. |
| `navigation_radius`, `costmap_resolution` | Nav2 costmap radius and resolution, in m. They set the narrowest allowed corridor. Required. |
| `tested_corridor_width` | Width the stack was tuned on. Narrower mazes give a warning. Required. |
| `lidar.range_max`, `lidar.mount_height` | The range sets the ground plane margin (range + 0.5 m). The mount height sets the minimum wall height (+ 0.05 m). |
| `stack_params` | Default `nav_params`, `slam_params`, `explore_params`, `ekf_params`, as `package://` or absolute paths. Required. |
| `launch_args` | Passed to `maze.launch.py`: `robot_model`, `robot_sdf`, `robot_bridge_config`, `robot_urdf`. Blank keeps the Burger. |
| `spawn.z`, `env` | Spawn height in m, and launch environment variables such as `TURTLEBOT3_MODEL`. |

`bridge_config` names the robot's `ros_gz_bridge` config (`package://` or an
absolute path). The suite passes it to the launch as `robot_bridge_config`; for
the Burger it is the launch file's own default.

To add a robot:

1.  Prepare its files: the Gazebo model, a `ros_gz_bridge` config, and stack
    parameter files tuned for it.
2.  Copy `burger.yaml` to `~/maze_experiments/robots/<name>.yaml` and change the values.
3.  Check it with `maze_experiments config check` on a configuration that uses it.
    Missing parameter files and profile errors are reported.
4.  Set `robot_profile: <name>` in a configuration.

**Why:** the suite must not be tuned to one robot. The results depend on the robot
model and the stack's settings, so the profile records both (FR1).

## Troubleshooting

### WSL clock jumps

**Symptoms:** the run dialog warns that `systemd-timesyncd` is active under WSL, or
that the journal has "Clock change detected" entries. Runs end as `infra_failure`
(reason `clock_jump`). SLAM stalls.

**Cause:** when the Windows clock is off, WSL and `systemd-timesyncd` both set the
time, and each backward jump stalls SLAM. If Windows sleeps, the whole VM pauses.

**What the suite does:** it detects a host suspend (a gap over 30 s between the
manager's ticks) and a clock jump (the wall clock steps by more than 2 s, or the
journal has a clock-change entry). The runs active then end as `infra_failure` and
are repeated. With `--clock-jumps flag`, or the matching check box, they are only
marked.

**Fix:** see the "WSL clock jumps" section of the repository `README.md`.

### llvmpipe (software rendering)

When the renderer is `llvmpipe`, Gazebo renders the lidar on the CPU and every run is
much slower. The suite still runs, with fewer runs at once, and it warns. Check with
`python3 tools/check_gpu.py` in the repository. If it reports `llvmpipe`, run
`./setup.sh` again, as the README's common problems section describes.

### No free ROS domain ID

"No free ROS domain ID (1-101); waiting for one" means every ID is held. The suite
waits, and starts the next run when an ID is free. The holds are files in
`~/.cache/maze_experiments/leases/`. A hold whose processes are gone is removed
automatically. If the wait does not end, check that folder and cancel experiments
you do not need.

### Processes left after SIGKILL

"Run ...: N process(es) survived SIGKILL" means some processes of a run could not be
stopped. The run's domain ID and Gazebo partition stay reserved until they are gone.
To list the suite's processes:

```bash
grep -l -a MAZE_EXPERIMENTS_RUN /proc/[0-9]*/environ 2>/dev/null
```

The numbers in the paths are the process IDs. If they do not go away, stop them with
`kill -9 <pid>`. Do not use `gzkill`: it stops every Gazebo server on the PC.

### A Gazebo server is already running

A Gazebo server you started by hand, in the default partition, does not stop suite
runs. Each run checks only its own partition. A server left over from an interrupted
suite run is stopped when the experiment resumes.

### The disk would run out

The run dialog compares the estimated disk use with the allocation and the free
disk, and shows an error when the estimate is too large. Allocate more, use
`metrics_only`, use `failures` or `none` retention, or free space. During a run the
experiment pauses with reason `disk` if the projection exceeds the allocation or the
free space.

### The runs are slow, or maze generation failed

Low RTF on the run screen means fewer runs at once. When even one run cannot keep the
minimum, the suite says so, runs one at a time, and flags them with `low_rtf`.

Discarded mazes are logged as `maze_discarded` in `manager.jsonl`, with their reasons.
If a set cannot give the mazes it needs, the experiment becomes interrupted and the
error is logged there. Change the maze parameters in a modified configuration.

### Other messages

*   "maze_experiments is not built into the workspace", or "maze_sim is not installed":
    run `./setup.sh` and open a new terminal.
*   "The data directory ... is on the Windows drive": use a folder such as
    `~/maze_experiments`, with `--data-dir`.
*   "needs a newer maze_experiments": update the repository and rebuild.
*   "Could not read the system journal", or "The renderer could not be determined":
    the clock or renderer check could not run on this PC.

## Acceptance test results

Run on 2026-11-26 on the development PC: AMD Ryzen 5 5600G (12 logical CPUs in WSL), 15.5 GB
WSL memory, NVIDIA RTX 4070 through D3D12, Ubuntu 24.04 under WSL 2.

**Important caveat.** Throughout the tests, this PC's WSL clock was stepped back by about 0.9 s
every 31 s ("Clock change detected" in the journal), with `systemd-timesyncd` already disabled
and the Windows clock not synchronized (`w32tm /query /status`: "Local CMOS Clock", 0.5 s off
internet time). With the specification's default (BR13), every run would have ended
`infra_failure` and been repeated until its retries ran out. The tests were therefore run with
`--clock-jumps flag` (runs are kept and flagged). The suite itself works under these conditions,
but the **robot results below are not representative of the robot stack**: clock jumps stall
SLAM, and a single run here already dipped below the minimum RTF of 0.8. Repeat the robot
measurements on a PC whose clock is stable.

| Test (5.0) | Result |
|---|---|
| 1. Generator | Passed. The property test generated 1000 mazes over random valid parameters and confirmed AR3, AR4, AR6, AR8 and AR9 for each (about 13 s). For 20 seeds, `make_maze.py`'s `build()` on the generated layout gave identical walls (0.5 m / 0.02 m geometry; see the design document's limits). A 20 x 20 maze with its checks took 17 ms (ANR2). |
| 2. One run | Passed. 5 x 5 m, 2-exit mazes ran to outcomes (`complete`, `incomplete`, `return_failed`, `stalled`, `timeout`). A run folder opened in `maze_monitor --replay` (9:07 long, 304 events) and `tools/drift_report.py` read it; the suite's localization metrics matched its output (SLAM max error 54.0 cm against 54.2 cm, EKF 18.9 cm in both). |
| 3. Isolation | Passed. With two concurrent runs (domains 1 and 2, their own partitions) and an interactive `ros2 launch maze_sim maze.launch.py` in domain 0, each environment saw exactly one publisher of `/clock`, `/scan`, `/ground_truth/poses` and `/map`, one `slam_toolbox` and one `ground_truth_bridge`; `gz model --list` in each partition showed only that partition's world (the repository maze with 18 walls for the interactive one, the generated mazes for the runs). |
| 4. Stopping | Passed. Cancelling the experiment stopped both runs on SIGINT within 5 to 8 s, recordings closed. Afterwards no process carried the experiment's run marker; the remaining `gz sim`, SLAM and Nav2 processes were the interactive simulation's, whose `/clock` kept advancing. |
| 5. Resume | Passed. The manager of a three-run experiment was killed with SIGKILL during the second run. On the next `maze_experiments status`, the 22 leftover processes of that run were stopped (by environment marker and start time) and the experiment was marked interrupted. After `resume`, the experiment completed with exactly three robot results, one per planned run; the killed attempt is kept as `s00-m001-r00-a1` with outcome `interrupted` (reason `manager_crashed`) and did not use a retry. |
| 6. Timeouts | Passed. With a sim timeout of 60 s, the run was stopped at 60.3 s of sim time with outcome `timeout`. A 3 x 3 m maze whose two exits were walled off by hand ended `incomplete` (explore_lite abandoned frontiers; 0 of 2 exits; return failed). |
| 7. Scaling | Passed. With 12 CPUs allocated, the calibration (4.6 CPU cores and 1.7 GB per run) allowed 2 at once. The controller raised the target to 2 only after the first run had held RTF >= 0.8 for 60 s. Both runs then fell to an RTF of about 0.4; after 30 s below 0.8 the target was lowered to 1 and new starts were held back. (Since then, a level that fails twice is not tried again in the experiment.) |
| 8. Full experiment | Passed (functionally). Use case 2.3.1 at 10 runs: the configuration was created in the configuration editor and started from the run dialog (GUI driven programmatically, offscreen, with screenshots); the GUI was closed while it ran and reopened on WSLg, showing the live run screen (BNR5). The command line followed it (`status`), printed the report and exported Markdown (18 KB), CSV and JSON. Results: mission success 0 of 10 (95% CI 0–28%); outcomes 2 complete, 2 incomplete, 4 return_failed, 2 stalled; 1 startup_failed attempt repeated; median coverage 22%; SLAM median max error 61 cm; mean RTF 0.58, every run below 0.8. See the caveat above. |
| 9. Sharing | Passed. An experiment folder (206 MB) was copied into an empty data directory standing in for a second account and imported; its missing configuration was recreated from the copy inside the experiment, and its results opened in 0.15 s. |

Measured non-functional targets on this PC:

| Requirement | Measured |
|---|---|
| ENR1: processing a run < 1/5 of its wall time | 7.4 s for a 184 s run (4%) |
| BNR3 and CNR2: manager (including sampling) < 5% / 2% of a core | 0.82% of one core over 2 minutes while supervising a run |
| GNR2: results of a 100-run experiment open < 5 s | 0.12 s (synthetic 100-run experiment; building its results took 2.4 s) |
| ER8: Markdown export < 200 KB for 100 runs | 112 KB |
| DNR2: listing 1000 experiments < 2 s | unit test, passes |

