# Maze monitor design

This document describes the architecture of `maze_monitor`, the debugging and
test monitor for the maze robot, and explains why it is built the way it is.
For instructions on using the monitor, see the [user manual](user-manual.md).

## Goals

The monitor helps you understand what the robot is doing and why something
went wrong, without reading streams of raw numbers. It must do the following:

*   Manage simulation runs: start, stop, and restart the simulation with
    settings chosen in the window (launch arguments, single parameters, and
    presets), so nobody has to type launch commands while testing.
*   Let you pause and resume exploration and drive the robot with the
    keyboard.
*   Show live data, and still work when the simulation is started from a
    terminal.
*   Offer a **Raw** view with the unprocessed values and an **Analysis** view
    that turns those values into plain-language findings and pictures.
*   Let you show or hide each ROS 2 topic.
*   Save every run to disk so that you can replay it later with a timeline:
    pause, play, skip, change speed, and jump to problems.
*   Manage saved runs: list, search, rename, annotate, star, export, and
    delete them.
*   Stay minimal. Color appears only where it separates or classifies
    information.

### Non-goals

*   Steering the robot's behavior. The monitor publishes only what you ask
    for with the robot controls: pause and resume for explore_lite, a cancel
    for Nav2 goals, and keyboard speed commands. It never sends goals of its
    own.
*   Replacing `ros2 launch`. The monitor runs `maze.launch.py` as it is; the
    launch file stays the single description of a simulation run.
*   Replacing RViz. The monitor draws a simple 2D map for context, not a 3D
    scene.
*   Replacing `ros2 bag`. Runs are standard rosbag2 recordings, so the
    standard tools still work on them.

## Overview

The monitor is one Python process with two threads:

*   The **ROS thread** runs an `rclpy` node. It discovers topics, subscribes to
    them, writes every message to the run file, and hands each message to the
    GUI thread through a queue.
*   The **GUI thread** runs Qt. It feeds messages into the timeline and the
    analyzer, and redraws the visible view ten times per second.

```
             ROS thread                          GUI thread
 ┌────────────────────────────────┐   queue   ┌──────────────────────────────────┐
 │ RosBridge                      │ ───────▶  │ Timeline ◀── Analyzer            │
 │  - topic discovery (every 2 s) │ (topic,   │   tracks    series, events       │
 │  - QoS matching                │  bytes,   │      │                           │
 │  - raw subscriptions           │  time)    │      ▼                           │
 │  - Recorder ──▶ run bag (.db3) │           │ Views render (timeline, cursor)  │
 └────────────────────────────────┘           └──────────────────────────────────┘
                                                      ▲
 Replay:  run bag ──▶ RunLoader ──▶ same Timeline + Analyzer (no ROS thread)
```

### The central idea: views are a function of (timeline, cursor)

Every view draws the state of a `Timeline` at a single point in time, called
the *cursor*. The timeline holds everything that is known about a run:

*   **Tracks**: every received message, per topic, as serialized bytes with its
    receive time.
*   **Series**: derived values over time, such as speed, wall clearance,
    heading, and distance to goal.
*   **Events**: findings with a start time, an optional end time, a severity,
    and a plain-language explanation. Examples are a likely collision, a stuck
    robot, a failed plan, or a topic that went silent.

Live mode and replay mode differ only in where the timeline comes from and
how the cursor moves:

| Mode   | Timeline source                       | Cursor                                      |
|--------|---------------------------------------|---------------------------------------------|
| Live   | Grows as messages arrive from ROS 2   | Follows the newest time, unless you pause   |
| Replay | Built once from a saved run file      | Moved by the transport controls             |

**Why:** One rendering path serves both modes, so what you see in a replay is
exactly what you would have seen live. It also gives live mode a free
feature: you can pause the live view and scrub back to look at something
that just happened while recording continues in the background.

## Components

The package uses this layout:

```
maze_monitor/
├── maze_monitor/
│   ├── main.py            Entry point, command-line options, shutdown handling
│   ├── config.py          Thresholds and recording rules, with YAML overrides
│   ├── launcher.py        Starts and stops ros2 launch as a process group
│   ├── run_settings.py    Settings of a run, its launch command, presets
│   ├── param_files.py     Parameter files: defaults, overrides, per-run copies
│   ├── teleop.py          Held keys to a speed command
│   ├── ros_bridge.py      rclpy node: discovery, QoS matching, subscriptions,
│   │                      robot control publishers
│   ├── clock.py           Steady receive clock (see RosBridge)
│   ├── recorder.py        rosbag2 writer and reader for runs
│   ├── run_store.py       Run folders, metadata, listing, export, deletion
│   ├── session.py         A timeline plus its analyzer; loads runs for replay
│   ├── timeline.py        Tracks, series, events, and lookups by time
│   ├── analysis/
│   │   ├── analyzer.py    Routes messages to interpreters; runs detectors
│   │   ├── interpreters.py  One interpreter per kind of data (see below)
│   │   ├── geometry.py    Angles, quaternions, 2D poses
│   │   └── diagnosis.py   Builds the plain-language status from a snapshot
│   └── gui/
│       ├── main_window.py Window layout, run control, modes, menus, shortcuts
│       ├── run_panel.py   Run settings form, Start, Stop, Restart, presets
│       ├── params_dialog.py Parameter editor
│       ├── robot_control.py Exploration buttons, keyboard driving
│       ├── launch_log.py  Launch output panel
│       ├── topics_panel.py Topic list with show and hide check boxes
│       ├── raw_view.py    Raw view
│       ├── analysis_view.py Analysis view (cards)
│       ├── widgets.py     Heading dial, lidar view, map view, plots, bars
│       ├── transport.py   Timeline bar and playback controls
│       ├── events_panel.py Event list
│       ├── run_manager.py Saved runs dialog
│       └── style.py       The color palette and its meaning
├── config/monitor.yaml    Default settings
├── docs/                  This document and the user manual
└── test/                  Unit tests that run without a ROS graph
```

### RosBridge (ROS thread)

`RosBridge` owns an `rclpy` node named `maze_monitor` and spins it in a
background thread.

*   **Discovery.** Every two seconds, the bridge lists the topics in the ROS
    graph and subscribes to new ones. It includes hidden action topics such as
    `/navigate_to_pose/_action/status`, because Nav2 goal results only appear
    there.
*   **Raw subscriptions.** The bridge subscribes with `raw=True`, so messages
    arrive as serialized bytes. The recorder writes the bytes as they are,
    which costs no deserialization.
*   **QoS matching.** For each topic, the bridge reads the QoS settings of the
    publishers and matches them, as `ros2 bag record` does. For example, it
    subscribes to `/map` as *transient local* so that it receives the latest
    map even if the map was published before the monitor started. A fixed QoS
    would silently miss such topics or never connect to *best effort*
    publishers.
*   **Rate limits.** Some topics are much faster than debugging needs. The
    Gazebo `/clock` can publish 1000 times per second. The configuration caps
    such topics (for example, `/clock` at 10 Hz and `/joint_states` at 10 Hz)
    before recording.
*   **Size guard.** A first test recorded 530 MB in 150 seconds. Almost all of
    it came from two DWB debug topics, `/evaluation` and `/marker`, at about
    1 MB per message. The default configuration excludes them, and any topic
    whose messages are larger than 256 KB is recorded at most every two
    seconds. This keeps unknown debug topics from filling the disk.
*   **Steady receive clock.** The bridge stamps each message with a clock that
    reads the wall time once and then advances with the monotonic clock. Under
    WSL, the wall clock steps backward by 0.5 to 1.5 seconds about every 30
    seconds when WSL resyncs it with Windows. rosbag2 orders messages by these
    stamps, so a stepping clock interleaves older and newer messages in the
    recording. During testing, this made odometry appear to jump back and forth
    by 7 cm. The steady clock removes the problem.

### Recorder

The recorder writes each run with `rosbag2_py` in the **SQLite 3** storage
format.

**Why rosbag2:** Runs stay compatible with the standard tools. You can play a
run back into ROS with `ros2 bag play`, inspect it with `ros2 bag info`, or
open it in other ROS tools, without any converter.

**Why SQLite 3 instead of the default MCAP:** The monitor often stops
abruptly: the launch is interrupted with Ctrl+C, WSL shuts down, or a process
is killed. A test during development showed the difference:

| Storage | Closed normally | Process killed during recording               |
|---------|-----------------|-----------------------------------------------|
| MCAP    | Readable        | Unreadable, and `ros2 bag reindex` also fails |
| SQLite  | Readable        | Readable; all written messages present        |

A debugging tool must keep the data of the run that went wrong, which often is
the run that crashed. SQLite keeps it. The cost is a larger file, which is
acceptable for runs of a few minutes.

### Run store

Each run is one folder in the runs directory (default `~/maze_runs`):

```
~/maze_runs/
└── 2026-11-12_14-03-27_explore/
    ├── run.json        Metadata: name, label, times, notes, tags, star, summary
    └── bag/            rosbag2 recording (metadata.yaml and bag_0.db3)
```

`run.json` holds data that the run manager needs to list runs quickly,
without opening each recording:

*   Name, label (for example, `explore`), launch arguments, start time, and
    duration.
*   User data: notes, tags, and a star.
*   A summary: distance driven, number of errors and warnings, goals
    succeeded and failed, and the share of the map explored.

The monitor rewrites the summary every 10 seconds while recording and once
more when the run ends. If the monitor crashes, the next replay of the run
recomputes the summary from the recording and saves it.

**Why one folder per run:** You can copy, zip, or delete a run with ordinary
file tools, and a damaged run never affects the others.

### Timeline

The timeline stores three kinds of data:

*   `TopicTrack` keeps receive times and serialized bytes for one topic. It
    deserializes a message only when a view asks for it, and caches the most
    recent results. Keeping bytes instead of Python objects uses about a tenth
    of the memory.
*   `Series` keeps time and value pairs. Values can be numbers or text (for
    example, the Nav2 goal state). A lookup returns the last value at or
    before the cursor.
*   `Event` keeps a start time, an end time for conditions that last (for
    example, "close to a wall from 41.2 s to 43.0 s"), a severity, a
    category, a title, an explanation, and the robot position at the start.

All time values are seconds since the start of the run, based on the time the
monitor received each message. The recording stores the same receive times,
so a replay rebuilds the exact same timeline.

### Analyzer

The analyzer turns messages into series and events. It holds a set of
*interpreters*, one per kind of data. Each interpreter reads specific message
types, updates its state, writes series, and raises events. Detectors that
depend on time passing (for example, "no movement for 10 seconds") also run on
a periodic `tick`.

| Interpreter   | Input topics                                   | Produces |
|---------------|------------------------------------------------|----------|
| Pose          | `/odom`, `/tf` (`map` → `odom`)                | Position and heading in the map frame, distance driven, localization jumps |
| Stamps        | `/odom`, `/scan`, `/imu`                       | Header timestamps that go backward (two publishers on one topic) or restart (simulation reset) |
| Motion        | `/cmd_vel`, `/odom`                            | Commanded and actual speeds, blocked-robot, stuck, spinning, and oscillation detection |
| Clearance     | `/scan`                                        | Clearance per sector, nearest obstacle, close-to-wall and contact detection, wheel-slip detection |
| Impact        | `/imu`                                         | Sudden horizontal acceleration, classified as a bump (wall in the direction of travel) or a jolt (abrupt stop in open space) |
| Navigation    | Nav2 action status and feedback, `/plan`, `/behavior_tree_log` | Goal state, distance remaining, path length, replans, recoveries, planner and controller failures |
| Exploration   | `/map`, `/explore/frontier_states` (or the `/explore/frontiers` marker colours in older runs), `/explore/exits`, `/explore/status` | Known area, growth rate, frontiers by state, probable and confirmed exits, stalled exploration |
| Logs          | `/rosout`                                      | Warnings and errors from all nodes. Messages from the same node that differ only in numbers are merged into one event with a count, so 80 "worldToMap failed: mx,my: 30,11" lines become one. |
| Health        | All topics, `/clock`                           | Message rates, silent topics, simulation real-time factor |

The interpreters recognize message types, not only topic names. For example,
the motion interpreter accepts both `geometry_msgs/msg/Twist` and
`geometry_msgs/msg/TwistStamped` on `/cmd_vel`, so it works with both the
simulation and robots that use unstamped commands.

#### How the detectors decide

Each detector uses evidence from more than one source where possible, because
a single signal often misleads. The thresholds live in
`config/monitor.yaml`.

*   **Close to a wall:** the lidar clearance (range minus robot radius) is
    below 5 cm. Below 1.5 cm, the monitor reports contact.
*   **Blocked:** the robot is told to move at least 0.05 m/s, but for one
    second it moves at less than 30% of that speed. If the clearance in the
    direction of travel is also small, the monitor reports a likely collision.
*   **Wheel slip:** odometry says the robot drives toward a wall less than 1 m
    away, but the lidar distance to that wall does not shrink. Odometry
    in Gazebo comes from the wheels, so a robot pushing against a wall can
    report motion that never happens. This detector catches that case.
*   **Stuck:** a goal is active, but the robot moved less than 10 cm in the
    last 10 seconds.
*   **Spinning:** the robot turned more than 360° within 15 seconds while
    moving less than 15 cm.
*   **Oscillation:** the turn direction of the command flipped at least six
    times within five seconds.
*   **Bump or jolt:** the IMU measures more than 6 m/s² horizontally. It is a
    bump only if a wall is less than 5 cm away in the direction of travel, or
    the robot touches one. Otherwise it is a jolt: an abrupt stop or start, for
    example when Nav2's collision monitor halts the robot. The first version
    reported every spike as a bump; in the test maze, corridors leave only
    about 14 cm of clearance, so a plain "wall nearby" test was always true.
*   **Timestamps go backward:** a message on `/odom`, `/scan`, or `/imu` is
    stamped earlier than the previous one. Two nodes publish the topic, so
    other nodes see the robot in two states at once. The pose interpreter
    ignores the older stream so that the distance stays correct.
*   **Localization jump:** the SLAM correction (`map` → `odom`) changed by more
    than 10 cm or 5° at once. A jump means the map position of the robot was
    wrong before or after it, which can make the robot drive into walls that
    the map shows in a different place.
*   **Goals succeed without moving:** five or more goals within 30 seconds
    each succeeded less than a second after they were sent. A test run showed
    explore_lite sending 740 such goals in 45 seconds: the frontier was
    already within the Nav2 goal tolerance, so every goal succeeded at once.
*   **Silent topic:** a topic that published steadily has sent nothing for
    five of its usual periods (at least one second).
*   **Slow simulation:** the simulation runs at less than half of real time for
    five seconds. Nav2 then misses its control deadlines, which looks like
    jerky or wrong motion.

**Why heuristics instead of ground truth:** The monitor must work the same
with the real robot, where no ground truth exists. Every detector uses only
the topics that the robot itself publishes.

### Diagnosis

`diagnosis.py` builds a short plain-language status from a *snapshot*: the
values of all series and the events active at the cursor. The status has the
following parts:

*   A headline: **OK**, **Warning**, or **Problem**.
*   What the robot does, for example "Driving forward at 0.18 m/s while
    turning left at 20°/s."
*   The goal, for example "Driving to a goal 1.3 m away (12 s, 1 replan,
    no recoveries)."
*   The surroundings, for example "Nearest wall: 0.21 m, front left."
*   Each active problem, with its explanation.

Because the diagnosis depends only on the snapshot, it is the same live and
in replay.

### GUI

The GUI uses Qt 5 through PyQt5 and draws its own widgets with `QPainter`.

**Why Qt 5 and PyQt5:** ROS 2 Jazzy Desktop already installs them for RQt and
RViz, so the monitor adds no dependencies. Qt runs under WSLg, like RViz.

**Why custom widgets instead of a plotting library:** The views need few and
simple drawings: lines, circles, a grid, and a cursor. Custom widgets redraw
in well under a millisecond, need no extra package, and look the same on
every computer. Matplotlib is too slow for redrawing ten times per second,
and pyqtgraph is not part of a ROS installation.

The main window has these areas:

*   **Toolbar:** the mode (live or replay), the **Raw** and **Analysis**
    switch, the angle unit (degrees or radians), recording controls, and the
    run manager.
*   **Topics panel (left):** every topic with its type, rate, message count,
    and a check box that shows or hides it.
*   **Center:** the raw view or the analysis view.
*   **Events panel (right):** all findings, filtered by severity. Selecting an
    event moves the cursor to its start.
*   **Timeline bar (bottom):** playback controls, the timeline with event
    marks, and the time.

#### Raw view

The raw view shows, for each visible topic, the message statistics (type,
rate, count, age of the last message, data rate) and the fields of the last
message at the cursor. Long arrays, such as the 360 lidar ranges, appear as a
summary (count, minimum, maximum, and mean) followed by the first values. A
second table lists every derived series with its raw value and unit.

#### Analysis view

The analysis view shows cards. Each card answers one question:

| Card          | Question it answers                                       |
|---------------|-----------------------------------------------------------|
| Status        | Is everything OK, and if not, what is wrong?              |
| Map           | Where is the robot, where has it been, where does it plan to go, and where did problems happen? |
| Heading       | Which way does the robot face, and which way is it turning? |
| Motion        | Does the robot do what it is told?                        |
| Surroundings  | How close are the walls, and in which direction?          |
| Navigation    | What is the goal, and how is Nav2 doing?                  |
| Exploration   | How much of the maze is known, and is exploration progressing? |
| Health        | Are all sensors and nodes publishing at their usual rates? |
| Plots         | How did selected values change over time?                 |
| Run summary   | What happened in the run so far?                          |

Each card lists the topics it reads. A card disappears when you hide all of
its topics.

#### Use of color

Color carries meaning and is used for nothing else. The same color means the
same thing everywhere:

| Color | Meaning |
|-------|---------|
| Red   | Error or danger: collision, contact, failed goal |
| Amber | Warning: close to a wall, slow simulation, stuck |
| Green | OK, or goal succeeded |
| Blue  | Intent: what the robot was told or plans (commands, planned path, goal) |
| Black | Reality: what the robot measured or did (odometry, pose, trail) |
| Gray  | No data, stale data, or unknown map area |
| Purple | Exploration frontiers (green ring: maze exit) |

The convention "blue is intent, black is reality" makes mismatches visible.
For example, a blue speed bar next to an empty black one means the robot is
told to move but does not.

### Replay

The run loader reads a recording in time order and feeds every message into a
new timeline and analyzer, adding the same periodic ticks that live mode
uses. This takes a few seconds for a typical run and shows a progress dialog.

**Why rebuild instead of storing the analysis:** The recording is the single
source of truth. When a detector improves, old runs benefit from it on the
next replay, and the stored data can never disagree with the recording.

Playback advances the cursor by elapsed wall time multiplied by the speed.
Seeking is instant, because every lookup is a binary search on the sorted
times of a track or series.

### Run control

The monitor is a long-lived process; simulation runs come and go under it.

**Starting.** `RunSettings` (`run_settings.py`) holds what the Run panel shows
and turns it into one command:
`ros2 launch maze_sim maze.launch.py name:=value … monitor:=false`. The
monitor then does the following, in order:

1.  It looks for leftover Gazebo servers and `maze_sim` launches and offers to
    stop them, because `maze.launch.py` refuses to start next to a Gazebo
    server.
2.  It creates the run folder and saves the settings, the command, and the
    launch arguments in `run.json`.
3.  It writes the parameter files of the run into `params/`
    (`param_files.py`): a copy with the overrides applied for each file with
    changes, and a plain copy of the others. The command names only the
    changed copies and files the user chose, so it stays short and the
    defaults still come from the launch file.
4.  It starts recording, then starts the launch (`launcher.py`).

`monitor:=false` keeps the launch from opening a second monitor. A launch
started from a terminal passes `--attach` to the monitor it opens; that
monitor records the run and disables Start and Stop.

**Why a subprocess instead of the launch API in-process:** `ros2 launch` runs
its own asyncio loop and signal handling and expects to own the process. A
child process keeps a crashing launch from taking the monitor with it, behaves
exactly like the command a developer types, and lets the monitor show and save
its output.

**Process group.** The launch starts in a new session, so it and everything it
starts (Gazebo, SLAM, Nav2, RViz) share one process group, and a Ctrl+C in
the monitor's terminal does not reach them directly. Stopping sends SIGINT to
the whole group, which is what Ctrl+C in a terminal does, so the launch shuts
its nodes down in its usual order. If the group is still alive after
`launch.stop_timeout` (20 s), it gets SIGTERM, and after `term_timeout` (5 s)
more, SIGKILL. The GUI timer polls the state; nothing blocks the window.

**Ending.** A run ends when the whole process group is gone. If the launch
exits by itself but leaves processes behind (a Gazebo server after a crash),
the monitor stops the rest of the group the same way, so the next start finds
nothing left over. It then stops the recording and saves in `run.json` how the
run ended (`ended_by`, `launch_exit_code`). A launch that exits with an error
within 15 s is marked `launch failed`. **Restart** waits for this before it
starts the next run.

**Output.** A reader thread reads the launch output, strips terminal colors,
writes it to `launch.log` in the run folder, and queues it for the **Launch
output** panel.

**Exploration.** Pause and resume publish `std_msgs/Bool` on
`/explore/resume`, which explore_lite handles: pausing cancels its Nav2 goal.
The state shown comes from the analyzer's `explore.status` series
(`/explore/status`) and from node discovery (`explore_node`, `bt_navigator`).
**Start exploring** runs `ros2 run explore_lite explore` with the run's
exploration parameters in a second process group, which ends with the run.

**Keyboard driving.** `KeyboardDriver` installs an application-wide event
filter while driving is on. It accepts the `ShortcutOverride` event of the
driving keys, so that the arrow keys and Space drive instead of triggering the
timeline shortcuts, and leaves keys alone while a text box has the focus. It
publishes `geometry_msgs/TwistStamped` on `/cmd_vel` at `teleop.rate_hz`
while a key is held, then three zero commands. It sends zero when the window
loses the focus. By default, starting to drive pauses explore_lite and
cancels all Nav2 goals (`/navigate_to_pose/_action/cancel_goal`), so that Nav2
and the keys don't both command the robot.

The monitor subscribes to its own control topics like any other, so a
recording contains the commands it sent.

### Shutdown

The monitor handles `SIGINT` and `SIGTERM`, which the launch system sends when
you press Ctrl+C. It stops a simulation that it started (see [Run
control](#run-control)) and waits for it, then stops the ROS thread, closes the
recording, and writes the final summary. Closing the window asks first and
does the same without blocking the window. If the process dies before that,
the recording is still readable (see [Recorder](#recorder)).

## Integration with the simulation

The monitor starts `maze_sim/launch/maze.launch.py` for each run (see [Run
control](#run-control)).

The launch file also starts a monitor when the `monitor` launch argument is
`true`, which is the default, for runs started from a terminal. It passes
`--attach`, a label that describes the launch mode (`explore`, `nav`, or
`manual`), and the launch arguments, which the monitor saves in `run.json`.

The `maze_sim` package declares `maze_monitor` as a dependency, so
`setup.sh` and the CI build include it.

## Limits

*   The analyzer uses receive times. If the simulation runs slower than real
    time, durations in the monitor are wall-clock durations. The **Health**
    card shows the real-time factor so that you can account for it.
*   The timeline keeps the whole run in memory. A one-hour run with the default
    topics uses a few hundred megabytes.
*   The lidar of the TurtleBot3 Burger cannot measure closer than 12 cm from
    its center, and the robot radius is about 10.5 cm. Contact with a wall at
    the side of the robot therefore often shows as missing lidar readings
    rather than as a short distance. The clearance interpreter treats a sector
    of missing readings next to a close wall as contact.

## Testing

The unit tests in `test/` cover the timeline lookups, the geometry helpers,
each detector with synthetic messages, the run store, and recording and
reading back a run, including a recording whose process was killed. For run
control, they cover starting and stopping process groups with stand-in
programs (Ctrl+C reaching children, escalation to SIGKILL, cleanup when the
leader exits by itself, finding leftovers), the launch command for each set
of settings, presets, parameter overrides and their type checks, and the
keyboard speed command. They run
without a ROS graph:

```bash
cd ~/ros2_ws/src/capstone/maze_monitor
python3 -m pytest test
```

The CI workflow runs the same tests after the build.

The monitor was also tested end to end with the simulation
(`explore:=true`): live recording, a clean shutdown on Ctrl+C, and replay of
the recording, including rendering both views offscreen at chosen times.
