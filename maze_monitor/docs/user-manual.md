# Maze monitor user manual

The maze monitor shows what the robot is doing while it runs, records every
run, and lets you replay a run to find out why something went wrong. Use it
when the robot runs into a wall, can't find a path, gets stuck, or behaves in
a way you don't understand.

This manual explains how to start the monitor, read its views, replay and
manage runs, and investigate common problems. For how the monitor works
inside, see the [design document](design.md).

## Before you begin

Build the workspace with `setup.sh` as described in the repository
`README.md`. The monitor is part of the workspace and needs no extra
installation.

## Start the monitor

### With the simulation

The monitor starts automatically with the simulation:

```bash
ros2 launch maze_sim maze.launch.py explore:=true gui:=false
```

The monitor window opens next to RViz and starts recording a new run.

The launch file accepts two arguments for the monitor:

| Argument         | Default | Meaning |
|------------------|---------|---------|
| `monitor`        | `true`  | Open the monitor. |
| `monitor_record` | `true`  | Save the run to disk. Set it to `false` to watch without saving. |

When you stop the launch with Ctrl+C, the monitor saves the run and closes. To
look at the run afterward, start the monitor on its own and open the run (see
[Replay a run](#replay-a-run)).

### On its own

To watch a system that is already running (for example, the real robot on the
same network), start the monitor by itself:

```bash
ros2 run maze_monitor maze_monitor
```

To look at saved runs without connecting to ROS 2, use `--no-ros`. For
example, to open the most recent run:

```bash
ros2 run maze_monitor maze_monitor --no-ros --replay latest
```

The monitor accepts these options:

| Option               | Meaning |
|----------------------|---------|
| `--replay RUN`       | Open a saved run. `RUN` is a run folder, a folder name in the runs directory, or `latest`. |
| `--no-ros`           | Don't connect to ROS 2. Only replay is available. |
| `--no-record`        | Show live data without saving a run. |
| `--label TEXT`       | Add a label to the run folder name, for example `wall-test`. |
| `--runs-dir DIR`     | Save and look for runs in `DIR` instead of `~/maze_runs`. You can also set the `MAZE_RUNS_DIR` environment variable. |
| `--config FILE`      | Load settings, such as thresholds, from a YAML file. See [Change thresholds](#change-thresholds). |

## The window at a glance

The window has five areas:

1.  **Toolbar** (top). It shows the mode: **LIVE** (in red while recording)
    or **REPLAY** with the run name. It also has the **Analysis** and **Raw**
    view buttons, the **Angles** unit, the recording buttons, and **Runs…**.
2.  **Topics panel** (left). Every topic, with its rate and status. Clear a
    check box to hide a topic.
3.  **View** (center). The **Analysis** view or the **Raw** view.
4.  **Events panel** (right). Everything the monitor noticed, such as
    collisions, failed goals, and warnings from nodes.
5.  **Timeline** (bottom). The whole run as a bar, with playback controls.

### What the colors mean

Colors carry meaning and are used for nothing else. The same color means the
same thing everywhere in the monitor.

| Color  | Meaning |
|--------|---------|
| Red    | Error or danger: collision, contact with a wall, failed goal. |
| Amber  | Warning: close to a wall, stuck, slow simulation. |
| Green  | OK, or goal reached. |
| Blue   | Intent: what the robot is told or plans. For example, speed commands, the planned path, and the goal. |
| Black  | Reality: what the robot measured or did. For example, odometry speed, position, and trail. |
| Gray   | No data, old data, or unknown map area. |
| Purple | Exploration frontiers. A green ring marks a maze exit (dashed: a probable exit, not yet confirmed). |

Blue next to black shows whether the robot does what it is told. For example,
a blue speed bar with no black bar under it means that the robot is told to
drive but doesn't move.

## Analysis view

The Analysis view turns the data into plain language and pictures. It shows
the state at the *cursor*, the black line on the timeline. In live mode, the
cursor follows the newest data.

### Status

The colored bar at the top gives the overall state: **OK** (green),
**Warning** (amber), or **Problem** (red), followed by the most serious
current problem. Under it, a few sentences describe the following:

*   What the robot does, compared with what it is told. For example, "The
    robot is standing still; it is told to be driving forward at 0.20 m/s."
*   The navigation goal, its distance, and how long the robot has been on it.
*   The exploration progress.
*   The nearest wall and its direction.
*   Each active problem, with an explanation of what it means.

### Map

The map shows the SLAM map with these overlays:

*   The robot as a circle with a line for its heading, and its trail in black.
*   The planned path and the goal (X) in blue.
*   Exploration frontiers in purple: the frontier cells and a dot at the
    centre. Postponed frontiers are gray and abandoned ones amber. A maze exit
    has a green ring labelled **exit**; a probable exit that the lidar saw
    no return through once has a dashed ring labelled **exit?**.
*   Lidar points in gray. Clear **Lidar** to hide them.
*   Event marks: amber and red dots where warnings and errors happened. A ring
    marks events that are active at the cursor. Click a dot to move the cursor
    to that event.

Scroll to zoom, drag to pan, and double-click or click **Fit** to see the whole
map. Select **Follow** to keep the robot centered. Select **Info events** to
also mark info events. Use the **Trail** list to show the whole trail or only
the last 60 or 10 seconds. The coordinates under
the mouse pointer appear in the lower-right corner.

### Heading

The dial shows which way the robot faces. The black line is the heading; 0
points along the map's +x axis and angles grow counterclockwise, as in RViz.
The blue arc shows how far the robot is told to turn in the next second, and
the black arc shows how far it actually turns. A dashed blue line points to the
goal.

To switch between degrees and radians, use the **Angles** list in the toolbar
or press Ctrl+U. The setting applies to every angle and turn rate in the
monitor.

### Surroundings

A top-down lidar view around the robot, with the robot facing up. The dashed
amber ring marks the warning clearance. Points turn amber when they are closer
than the warning clearance and red at contact. Red ticks at the robot edge
mark readings closer than the lidar can measure (12 cm from the center for the
TurtleBot3 Burger lidar). The clearance in each direction appears at the
edges. Scroll to change the range.

*Clearance* is the distance from the edge of the robot, not from its center.

### Motion

Two bars compare commanded speeds (blue outline) with measured speeds (black
fill), for driving and for turning. The sentence under the bars says whether
the robot does what it is told.

### Navigation

The Nav2 goal state, time on the goal, distance left, path length, goal
position, number of goals reached and failed, recoveries, route changes, and
planner failures.

### Exploration

The explore_lite status, known map area and its growth over the last 30
seconds, the number of frontiers, and how many frontiers explore_lite
postponed, abandoned, considers probable exits, or recognised as maze exits,
with the positions of the exits.

### Health

The message rate of the main topics, the simulation real-time factor, and the
number of warnings and errors that nodes logged. A topic turns amber when it
goes silent.

### Plots

Values over time, with the cursor as a black line. Warnings and errors shade
the background. Click a plot to move the cursor there.

*   To choose the values, click **Values…**. When you select a commanded value
    and its measured partner (for example, **Commanded linear speed** and
    **Actual linear speed**), they share one lane: blue for commanded, black for
    measured.
*   To change the time span, use the list next to **Values…**.

### Run summary

Time, distance, average speed, and the number of errors, warnings, and
collision events up to the cursor.

## Raw view

The Raw view shows the data without interpretation. Switch to it with the
**Raw** button or Ctrl+2.

*   The upper table lists each visible topic with its type, rate, message
    count, the age of its last message, and its data rate. Expand a topic to
    see every field of its last message at the cursor. Long arrays, such as
    the 360 lidar ranges, show their count, minimum, maximum, and mean,
    followed by the first values.
*   The lower table lists every value that the analyzer computes, unrounded,
    with its unit and age.

## Show or hide topics

In the **Topics** panel, clear the check box of a topic to hide it. Hidden
topics disappear from the Raw view, and Analysis cards whose topics are all
hidden disappear too. Map overlays follow their topics: for example, hiding
`/plan` removes the planned path from the map.

Hiding a topic changes only what you see. The monitor still records it, so you
can show it again in a replay.

*   To find a topic, type part of its name in **Filter topics**.
*   To show or hide all listed topics, click **Show all** or **Hide all**.
*   To go back to the default selection, click **Defaults**. By default, the
    monitor hides topics that are rarely useful for debugging, such as
    `/clock` and `/tf_static`.

The **Status** column shows one of these values:

| Status   | Meaning |
|----------|---------|
| OK       | Messages arrive. |
| idle     | No message in the last 3 seconds. Normal for topics that publish only while navigating, such as `/plan`. |
| silent   | A sensor topic stopped publishing unexpectedly. |
| waiting  | The topic exists, but no message has arrived yet. |
| no pub   | No node publishes the topic. |
| excluded | The settings exclude the topic from recording and analysis. |
| error    | The monitor can't subscribe, usually because the message type isn't available. |

Hold the pointer over a topic to see its message type, its message count, and
an explanation of its status.

The monitor remembers your selection the next time it starts.

## Events

The **Events** panel lists everything the monitor noticed, in time order.

*   Click an event to move the cursor to its start. The box under the list
    explains the event and shows its category, duration, and position.
*   Events after the cursor appear in light gray. Events active at the cursor
    appear in bold.
*   To filter, use the **Errors**, **Warnings**, and **Info** check boxes and
    the category list.
*   Repeated log messages are merged into one event with a count, for example
    "(×37)".

## Replay a run

Each run is saved automatically. To replay a run, do the following:

1.  Click **Runs…** in the toolbar (or press Ctrl+R).
2.  Select the run and click **Open replay**, or double-click it.

The mode changes to **REPLAY**. If the monitor is connected to ROS 2, it keeps
recording the live run in the background. To return to it, click **Back to
live**.

### Playback controls

| Control                 | Action |
|-------------------------|--------|
| Timeline bar            | Click or drag to move the cursor. Red and amber spans above the bar are conditions that lasted (for example, "close to a wall"). Ticks below the bar are single events. Hold the pointer over a mark to see what it is. |
| Play or pause           | Play from the cursor. At the end of a run, **Play** starts again from the beginning. |
| Back and forward        | Move by the **Step** size (default 5 s). |
| Start and end           | Move to the start or the end of the run. |
| **Step**                | The step size, from 0.1 s to 600 s. |
| **Speed**               | Playback speed, from 0.1× to 16×. |
| **Previous problem**, **Next problem** | Move to the previous or next warning or error. |
| **Live**                | In live mode, follow the newest data. |

In live mode, you can also pause and move back to look at something that just
happened. Recording continues. Click **Live** or press L to follow the newest
data again.

### Keyboard shortcuts

| Keys                   | Action |
|------------------------|--------|
| Space                  | Play or pause. In live mode, pause the view. |
| Left, Right            | Move back or forward by the step size. |
| Shift+Left, Shift+Right | Move back or forward by 0.1 s. |
| Ctrl+Left, Ctrl+Right  | Move to the previous or next problem. |
| Home, End              | Move to the start or end. |
| [ , ]                  | Play slower or faster. |
| L                      | Follow live data. |
| Ctrl+1, Ctrl+2         | Show the Analysis view or the Raw view. |
| Ctrl+U                 | Switch between degrees and radians. |
| Ctrl+R                 | Open the run manager. |
| Ctrl+O                 | Open a run folder. |
| F1                     | Open this manual. |

## Record runs

While recording, the toolbar shows **LIVE · recording** followed by the run
name, and the number of recorded messages appears on the right.

*   To save the current run and start a new one, for example after you reset
    the simulation, click **New run**.
*   To stop recording, click **Stop recording**. The finished run stays on
    screen as a replay. Live data continues without being saved; click **Back
    to live** to see it, and **Start recording** to record again.

The monitor records all topics except a few that are large or not useful for
debugging, such as images and Nav2 bond topics. It limits `/clock` to 10
messages per second.

## Manage runs

Open the run manager with **Runs…** or Ctrl+R. The table lists all runs with
their start time, duration, distance, number of errors and warnings, goals
reached and failed, explored area, label, tags, size, and status. Click a
column header to sort by it.

| Task | Steps |
|------|-------|
| Find a run | Type in the search box. It searches names, labels, tags, and notes. |
| Replay a run | Select it and click **Open replay**, or double-click it. |
| Rename a run | Select it and click **Rename…**. The folder name stays the same. |
| Write notes and tags | Select the run and click **Notes and tags…**. Write what you changed and what happened, so that you can compare runs later. |
| Keep a run | Select it and click **Star**. Starred runs are listed first, and deleting them asks for extra confirmation. |
| Share a run | Select it and click **Export .zip…**. |
| Open the run folder | Select it and click **Show folder**. |
| Delete runs | Select one or more runs and click **Delete…**. Deleting can't be undone. You can't delete the run that is being recorded. |

### Where runs are stored

Runs are stored in `~/maze_runs`, one folder per run:

```
~/maze_runs/2026-11-12_14-03-27_explore/
├── run.json   Name, label, launch arguments, notes, tags, star, and summary
└── bag/       The recording
```

The recording is a standard rosbag2 recording in SQLite 3 format, so other ROS
tools can read it. For example:

```bash
ros2 bag info ~/maze_runs/2026-11-12_14-03-27_explore/bag
```

To play a run back into ROS, for example to view it in RViz:

```bash
ros2 bag play ~/maze_runs/2026-11-12_14-03-27_explore/bag
```

If the monitor was stopped abruptly (for example, by a crash or by closing
WSL), the run is still readable. Its status shows **recording** until you open
it once; the monitor then repairs the summary and sets the status to
**complete (recovered)**.

## Investigate common problems

Each procedure starts in the run manager with the run that went wrong.

### The robot ran into a wall

1.  Open the run, then click **Next problem** until you reach a red
    **Collision** event, such as **Likely collision**, **Touching a wall**, or
    **Wheels turn but the robot does not move**.
2.  Click **Back** a few times (or set **Step** to 1 s) to watch the seconds
    before the collision.
3.  Check the following, in order:
    *   **Localization:** Is there a **Localization jump** event shortly
        before? If so, the robot did not know where it was. Its plan was made
        on a wrong position.
    *   **Plan:** On the map, does the blue path run through or along the wall?
        If so, the map or the costmap was wrong; check whether the wall
        appears in the map at that time.
    *   **Motion:** In the **Motion** card, is the robot told to drive toward
        the wall (blue bar) at that moment? If the command is zero but the
        robot still moves, the problem is in the simulation or the drive, not
        in Nav2.
    *   **Health:** Is the real-time factor low, or is there a **Simulation
        runs at …× real time** event? A slow simulation makes Nav2 react late.

### The robot doesn't find a path

1.  In the **Events** panel, select only the **Planning** and **Navigation**
    categories.
2.  Look for **Planner failed to find a path**, **Planner returned an empty
    path**, and **Goal failed** events.
3.  Click one and look at the map: is the goal (blue X) inside a wall or in
    gray (unknown) space? Is the passage to it narrower than the robot plus
    its safety margin?
4.  Look for **Route got longer** events. They show where the planner gave up
    a shorter way, usually because the map revealed a wall.

### The robot is stuck or spins in place

1.  Click **Next problem** until you reach a **Stuck**, **Spinning in place**,
    or **Oscillating** event.
2.  Look at the **Navigation** card: a growing number of recoveries means Nav2
    tried to free the robot and failed.
3.  Look at the **Surroundings** card: walls close on several sides mean that
    the robot is wedged in a narrow spot.
4.  Look for **Exploration postponed a frontier** and **Exploration gave up
    on a frontier** events. explore_lite postpones a frontier when Nav2 fails
    or after its `progress_timeout` (20 s) or `commit_timeout` (60 s), and
    abandons it after `max_frontier_attempts` (3) tries.

### Exploration stops early

1.  Look at the **Exploration** card at the end of the run: the status, the
    number of frontiers left, and the abandoned frontiers.
2.  If frontiers are left but the status is **exploration complete**, all
    remaining frontiers are maze exits or were abandoned. Click their events
    to see where they are. The event details say how an exit was found: by
    the lidar seeing no return through it from two positions, or by the
    map not growing around it while the robot was close. An exit in the
    middle of the maze found from close by means the map didn't fill in
    within `exit_watch_time` (raise it in `explore_maze.yaml`); one found by
    the lidar means the lidar looked through it and out of a real exit from
    two places (raise `exit_confirm_views`).
3.  An **Exploration stalled** event means that the map didn't grow for 60
    seconds while exploring.

### Motion is jerky or slow

1.  Look at the **Health** card for the real-time factor and for silent
    topics.
2.  Look in the **Events** panel for **Log** warnings such as "Control loop
    missed its desired rate". The count shows how often it happened.

## Event reference

The following table lists the events that the monitor reports, what they
mean, and where to look next.

| Event | Severity | Meaning | Look at |
|-------|----------|---------|---------|
| Likely collision: pushing into an obstacle | Error | The robot is told to drive, doesn't move, and a wall is right in front of it (or behind it when reversing). | The plan on the map, and the localization just before. |
| Touching a wall | Error | The lidar shows less than 1.5 cm clearance, or readings closer than the lidar can measure. | Surroundings card; the plan on the map. |
| Close to a wall | Warning | Clearance below 5 cm. | Whether the plan runs along the wall. |
| Wheels turn but the robot does not move | Warning | Odometry reports forward motion, but the wall ahead doesn't get closer. Odometry is wrong from here on. | Surroundings card; the robot is probably pressed against something. |
| Not following the speed command | Warning | The robot moves at less than 30% of the commanded speed, with no wall right ahead. | Health card (slow simulation), Motion card. |
| Not turning as commanded | Warning | The robot is told to turn in place but doesn't. | Surroundings card: the robot may be wedged. |
| Bump against a wall | Warning | Sudden acceleration with a wall in the direction of travel. | The seconds before the bump. |
| Jolt | Info | Sudden acceleration in open space: an abrupt stop or start. | Log events from `collision_monitor` at the same time. |
| Stuck | Warning | Less than 10 cm of movement in 10 s while navigating. | Navigation card (recoveries), Surroundings card. |
| Spinning in place | Warning | More than 360° of turning in 15 s while moving less than 15 cm. | Recovery events; whether the goal is reachable. |
| Oscillating | Warning | The turn command changed direction six or more times in 5 s. | The plan near walls; controller settings. |
| Localization jump | Warning | SLAM moved the map position of the robot by more than 10 cm or 5° at once. | Events right after it: plans made before the jump are wrong. |
| Timestamps go backward | Error | Two publishers on one topic, so nodes see the robot in two states. | `ros2 topic info <topic> --verbose`; leftover simulations (`gzkill`). |
| Clock restarted | Warning | Message time went back by more than 5 s: the simulation was reset. | Nothing, if you reset it on purpose. Consider starting a new run. |
| … went silent | Warning | A sensor topic stopped publishing. | Whether its node crashed (log events). |
| Simulation runs at …× real time | Warning | Gazebo is slower than real time for 5 s or more. | GPU use (`tools/check_gpu.py`), other programs. |
| Goal failed | Error | Nav2 aborted the goal. | The planning and recovery events before it. |
| Goals succeed without moving | Warning | Five or more goals in 30 s succeeded within a second. The goals are already within the goal tolerance, so Nav2 reports success at once, and the sender keeps sending them. | `xy_goal_tolerance` in `nav2_maze.yaml`; the frontier sizes in `explore_maze.yaml`. |
| Planner failed to find a path, Planner returned an empty path | Warning | No path to the goal. | The goal (blue X) on the map: inside a wall or unknown space? |
| Controller failed to follow the path | Warning | The controller found no safe velocities. | Surroundings card; obstacles on the path. |
| Recovery | Warning | Nav2 ran a recovery behavior (spin, back up, wait). | What happened just before. |
| Route got longer | Info | The planner switched to a longer route to the same goal. | The map: which passage turned out to be blocked. |
| Exploration postponed a frontier | Info | explore_lite couldn't reach a frontier for now (Nav2 failure or a timeout). It retries it once no other frontier is left. | The frontier position on the map; goal failures before it. |
| Exploration gave up on a frontier | Warning | explore_lite failed to reach a frontier `max_frontier_attempts` times and won't try it again. | The frontier position on the map; goal failures before it. |
| Exploration spotted a probable maze exit | Info | The lidar looked through most of a frontier and got no return, so nothing lies behind it within lidar range. A second look from another position confirms it, or the robot gets close and checks whether the map fills in. | Whether the position is an opening in the outer wall. |
| Exploration found a maze exit | Info | A frontier opens to the outside: the lidar saw no return through it from two positions, or the robot got close and the map around it stopped growing. explore_lite won't drive to it again. | Whether the position is really an opening in the outer wall. |
| Exploration stalled | Warning | The map didn't grow for 60 s while exploring. | Navigation card; whether the robot is stuck. |
| Log messages in brackets, for example `[controller_server] …` | Warning or error | A node logged a warning or error. Repeats are merged and counted. | The node named in brackets. |

## Change thresholds

The monitor decides what counts as "close to a wall" or "stuck" with
thresholds. To change them, do the following:

1.  Copy the default settings file:

    ```bash
    cp ~/ros2_ws/src/capstone/maze_monitor/config/monitor.yaml ~/my_monitor.yaml
    ```

2.  Edit the values in `~/my_monitor.yaml`. Each setting has a comment that
    explains it. For a robot other than the TurtleBot3 Burger, set
    `robot.radius` first; clearances depend on it.
3.  Start the monitor with the file:

    ```bash
    ros2 run maze_monitor maze_monitor --config ~/my_monitor.yaml
    ```

Replays use the thresholds of the monitor that opens them, so you can open an
old run again with new thresholds.

## Troubleshooting

**The window is wider than the screen.** With both side panels open, the
window needs about 1430 pixels. Close the **Topics** or **Events** panel with
the **View** menu, or drag the panel edges to make them narrower.

**The window doesn't open.** Check that RViz opens. The monitor uses the same
display. Under WSL, run `wsl --shutdown` in Windows PowerShell and open a new
Ubuntu terminal.

**A topic shows "no pub" or "waiting".** No node publishes it yet. Nav2
and explore_lite topics appear about 15 seconds after the launch.

**A topic shows "Message type … is not available".** The message package isn't
built or sourced. Open a new terminal, or run
`source ~/ros2_ws/install/setup.bash`, and start the monitor again.

**The Map card stays empty.** SLAM publishes the map about every 5 seconds,
and only after it receives lidar data. Check that `/scan` and `/map` show
**OK** in the Topics panel.

**Many "Close to a wall" warnings although the robot looks fine.** The
default `robot.radius` (0.105 m) fits the TurtleBot3 Burger. For a larger
robot, increase it; see [Change thresholds](#change-thresholds).

**The runs folder grows large.** Delete old runs in the run manager. Star the
runs that you want to keep first, so that you don't delete them by mistake.
