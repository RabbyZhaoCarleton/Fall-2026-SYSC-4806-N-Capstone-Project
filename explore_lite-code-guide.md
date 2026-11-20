# explore_lite changes: code guide

All new C++ is in **m-explore-ros2 (explore_lite)**, the third-party exploration package. The repository doesn't contain these sources. It stores the changes as five patches in `patches/m-explore-ros2/` on `feature/maze-monitor`, and `setup.sh` applies them on top of the pinned upstream commit `326cf8a`.

To get the patched files to read or show (line numbers below refer to these):

```bash
git clone https://github.com/robo-friends/m-explore-ros2.git && cd m-explore-ros2
git checkout 326cf8a
for p in ../capstone/patches/m-explore-ros2/*.patch; do git apply "$p"; done
```

Or open `~/ros2_ws/src/m-explore-ros2` after running `setup.sh`.

Size of the change: 15 files, about +2,000 / −110 lines.

| File | What | Patch |
|---|---|---|
| `explore/src/frontier_search.cpp` | distance around walls (+7 lines) | 0001 |
| `explore/include/explore/frontier_memory.h` | **new**: remembers frontiers and their state | 0002, 0004 |
| `explore/include/explore/exit_detector.h` | **new**: lidar test "is there anything behind this frontier?" | 0004 |
| `explore/include/explore/exit_watch.h` | **new**: "the map near the frontier stopped growing" timer | 0004 |
| `explore/include/explore/finish_check.h` | **new**: wait-and-recheck before stopping | 0005 |
| `explore/src/explore.cpp` / `explore.h` | main node: commitment, exits, finishing, return home (about +900 lines) | 0002–0005 |
| `explore_lite_msgs/msg/Frontier.msg`, `FrontierArray.msg` | **new** messages for `/explore/frontier_states` and `/explore/exits` | 0004 |
| `explore_lite_msgs/msg/ExploreStatus.msg` | adds `exploration_incomplete`, `return_failed` | 0005 |
| `explore/test/test_explore.cpp` | 29 new gtest cases | 0002–0005 |

Design choice to point out: the decision logic is in **small header-only classes with no ROS dependencies** (`FrontierMemory`, `exit_detector`, `ExitWatch`, `FinishCheck`). They take plain numbers and points, so they can be unit-tested without running ROS, Nav2 or Gazebo. `explore.cpp` only connects them to the costmap, TF, lidar and the Nav2 action client.

---

## 1. Frontier distance around walls (`frontier_search.cpp:49, 75, 82–85`)

Upstream already finds frontiers with a breadth-first search (BFS) that floods outward from the robot through free cells. The patch records how many steps the BFS took to reach each cell:

```cpp
std::vector<unsigned int> path_dist(size_x_ * size_y_, 0);
...
path_dist[nbr] = path_dist[idx] + 1;          // when a free neighbour is queued
...
new_frontier.min_distance = (path_dist[idx] + 1) * costmap_->getResolution();
```

BFS reaches cells in order of distance, so this is the shortest path length in grid steps. The cost function already uses `min_distance`, which previously was the straight-line distance, so frontier ranking now follows the route the robot can actually drive.

**Likely question:** *Why not Euclidean / A\*?* A straight line ignores walls. A per-frontier A\* would repeat work the BFS already does. This costs one array and one addition.
**Caveat:** it uses 4-connected steps, so diagonal routes are slightly overestimated (Manhattan-like). That's fine for ranking frontiers.

## 2. FrontierMemory: frontier states (`frontier_memory.h`)

Frontiers are recomputed from scratch on every search and change shape as the map grows. To remember them, the class stores a point and matches any new frontier whose centroid or any cell is within `frontier_match_radius` (0.3 m) of it (`matches()`, line 54).

States (line 26): `OPEN`, `EXIT_CANDIDATE`, `EXIT`, `POSTPONED`, `ABANDONED`. Each `Entry` holds its point, state, attempt count, and `exit_views`, the robot positions from which the lidar saw it open.

Key methods:
- `state()` (line 78): if a frontier matches several entries, the **most restrictive** state wins (EXIT > ABANDONED > CANDIDATE > POSTPONED > OPEN, `rank()` line 273). A frontier overlapping a known exit is never chosen as a goal.
- `observe()` (line 120): applies a lidar verdict.
  - `OPEN` adds a viewpoint, but only if it is at least `exit_view_separation` (0.3 m) from earlier ones, so turning in place doesn't count as a second look.
  - Enough views make the frontier an `EXIT`; fewer make it an `EXIT_CANDIDATE`.
  - `ENCLOSED` (walls seen behind it) clears the views and turns a candidate back to `OPEN`.
- `owner()` (line 174): a lidar verdict only applies to the remembered entry that **this** frontier is closest to, so a neighbouring frontier can't confirm or overrule another one's exit status.
- `set()` / `track()` (lines 232, 247): move the remembered point as the frontier drifts. An `EXIT` never moves or changes. A move larger than the match radius drops old lidar views, because they were of a different place.
- `reopenPostponed()` (line 260): used once nothing else is left.

## 3. Lidar exit detection (`exit_detector.h`)

The physical idea: a lidar ray that hits nothing returns `inf`. slam_toolbox leaves those rays out of the map, so the area outside a maze exit stays unknown and a frontier stays in the opening. If the rays pointing **through** a frontier come back empty, nothing lies behind it within 3.5 m, so it opens to the outside.

`scoreFrontier()` (line 342). For each frontier cell:
1. Compute the bearing from the sensor pose to the cell, convert it to a scan index (`(atan2 − yaw − angle_min) / angle_increment`), and wrap it for a 360° scan.
2. Skip cells beyond `range_max` or outside the field of view.
3. Classify the ray:
   - `no_return` if the reading is `inf`, ≥ `range_max`, or (real LDS driver) 0;
   - `returned` if it hit something **more than 5 cm behind** the cell;
   - otherwise the ray hit something before the cell (blocked), which says nothing.

`judge()` (line 399):
- `OPEN` if at least 5 points have no return and they make up at least 75% of the frontier's points;
- `ENCLOSED` if at least 5 points returned and fewer than 25% of the seen-through points had no return;
- `UNKNOWN` otherwise.

`Explore::observeExits()` (`explore.cpp:658`) supplies the scan:
- It keeps the last 20 scans (`scanCallback`, line 650).
- It walks back from the newest scan to find one whose `map → laser` transform is available at the scan's timestamp, because the SLAM transform often lags the scan. Scans older than 2 s are not used.
- It builds a `ScanView` and runs `scoreFrontier`/`judge` on every frontier.
- When the robot is within `exit_confirm_distance` (0.25 m, path distance), one view is enough (`close ? 1 : exit_confirm_views_`, line 712), because from close up the rays fan out over a wide angle.

## 4. Exit watch (`exit_watch.h`, `explore.cpp:551–648`)

This was added in `3dd8d3e`. The lidar check can be fooled when an inner doorway lines up with an exit. So exit candidates are **no longer saved for last** (`makePlan`, `explore.cpp:361–373`: `pick_open` accepts `OPEN` or `EXIT_CANDIDATE`). Getting close decides instead:

- `visibleDistance()` (line 578): the straight-line distance to the nearest frontier cell with **no wall between** it and the robot. It walks the costmap cells with Bresenham's line algorithm (lines 597–616) and treats cells at or above `INSCRIBED_INFLATED_OBSTACLE` as walls.
- `countUnknown()` (line 551): counts unknown cells in a 0.5 m disc around the frontier. Cells off the edge of the map count as unknown.
- `ExitWatch::update()` (line 54) works like a watchdog timer:
  - beyond 0.6 m it stops watching;
  - when it starts watching, it records the time and the unknown-cell count;
  - if the unknown area shrinks by at least `min_growth` (0.02 m² in cells), the lidar is still mapping, so it **restarts the timer**;
  - after 6 s with no shrinking, the frontier is an exit.
- It watches the area around the frontier's position **when watching began** (`center()`). Otherwise the frontier moving as it gets explored would reset the count.

Why it works: near an inner frontier the lidar fills the space behind it within one or two map updates (`map_update_interval` is 2 s). Behind an exit nothing ever fills in.

## 5. Committing to a frontier (`explore.cpp:423–523, 852–924, 1090–1158`)

Upstream re-ranked frontiers every cycle and switched goals whenever another one scored better, which caused flip-flopping. Now:

- `commitTo()` (line 852) records the goal, start time, attempt number and starting distance, then sends the Nav2 goal.
- Each plan cycle, `continueCommitment()` (line 423):
  1. Finds the committed frontier again with `findCommitted()` (line 525), which matches by position.
  2. If it became an `EXIT` (`leaveForExit`) or the exit watch fires, the robot lets go and picks another.
  3. If it's gone: explored. If its cell is still unknown, a wall now cuts it off, so it is postponed.
  4. Progress is measured with the **path distance**. No progress for `progress_timeout` (20 s), or `commit_timeout` (60 s) in total, calls `giveUp()`.
- `reachedGoal()` (line 1090):
  - Nav2 `ABORTED` retries the same frontier up to `nav2_retries` (2) times after 2 s, because a late TF can make Nav2 fail immediately.
  - After that, `giveUp()` (line 872). If the robot is within 0.4 m it judges the frontier as reached (`startSettling`); otherwise `postponeOrAbandon()` (line 893) postpones it, and abandons it after 3 attempts.
  - It also ignores results from goals it already replaced (`goal_id != active_goal_id_`, line 1094).
- **Settling** (`startSettling`, line 912, and the `settling_` branch at line 435):
  - after "success", wait `exit_settle_time` (4 s) for the map to catch up;
  - if the frontier is still there and the robot is within 0.4 m **path distance**, it is an exit;
  - otherwise postpone it.

  The path-distance check is the 0003 fix. Nav2 sometimes "succeeds" at a substitute point on the far side of a wall.

## 6. Two timers (`explore.cpp:212–220`, `742–778`)

- `exploring_timer_` → `makePlan()` at `planner_frequency` (0.15 Hz, about every 6.7 s): chooses goals.
- `exit_timer_` → `updateFrontiers()` at `exit_check_frequency` (2 Hz): searches frontiers, runs the lidar check and the exit watch, and publishes `/explore/frontier_states`. It only calls `makePlan()` when the committed frontier turns out to be an exit, so the robot turns away within half a second instead of waiting up to 7 s.

Both run on ROS 2's single-threaded executor, so they never run at the same time and don't need locks between them (the costmap still has its own mutex).

## 7. Stopping and returning home (`finish_check.h`, `explore.cpp:398–421, 959–1088, 1168–1213`)

- In `makePlan()`, if nothing is left to pick (no `OPEN` or `CANDIDATE` frontier, even after re-opening postponed ones), it calls `noFrontiersLeft()` (line 398).
- `FinishCheck` (line 120) only returns true once that has held for 4 s. A timer on the **node clock** (so it follows sim time) runs `makePlan()` again at the end. If a frontier shows up on the recheck, exploration continues (line 386).
- `finishExploring()` (line 959):
  - lists any `ABANDONED` frontiers and publishes `exploration_incomplete` with their coordinates, or `exploration_complete` if there are none;
  - then calls `stop(true)`.
- `stop()` (line 1168) contains the 0003 race-condition fix. The return-home goal is sent **inside the cancel's response callback**. Before, the cancel and the new goal were sent together, the cancel could arrive at Nav2 after the new goal and cancel it too, and the robot stood still.
- `returnToInitialPose()` / `retryReturn()` (lines 1013, 1059):
  - a rejected, cancelled or failed return is retried up to `nav2_retries` times;
  - after that it publishes `return_failed`.
- `endNavigation()` (line 994) cancels the timers and, if `exit_when_done`, calls `rclcpp::shutdown()` one second later so the last status message gets out. In the launch file, `shutdown_when_done:=true` attaches `Shutdown()` to explore_lite's `on_exit` to stop the whole simulation.

## 8. Tests (`explore/test/test_explore.cpp`)

29 new gtest cases. Useful ones to mention:
- `ExitDetector`:
  - `frontierWithNoReturnBehindItIsOpen`, `frontierWithWallBehindItIsEnclosed`, `frontierBehindAWallTellsNothing`
  - `usesTheSensorHeading`, `zeroReadingsCanMeanNoReturn`
- `FrontierMemory`:
  - `exitIsConfirmedFromASecondPosition`, `wallsBehindACandidateReopenIt`
  - `neighbourCannotTakeOverAnExit`, `exitWinsWhenFrontiersOverlap`, `bigMoveDropsLidarLooks`
- `ExitWatch`: `quietAreaNearAFrontierIsAnExit`, `growingMapRestartsTheTimer`, `onlyWatchesFromNearby`
- `FinishCheck`: `waitsBeforeStopping`, `frontierFoundOnRecheckKeepsExploring`

They run with `colcon test --packages-select explore_lite`. CI builds the patched package but only runs the Python monitor tests, so these aren't run automatically yet.

---

## Non-C++ changes worth knowing

- **`maze_sim/launch/maze.launch.py`**:
  - `spawn_robot()` builds the EKF setup: its own Gazebo bridge without the wheel-odometry TF, a static TF for the IMU frame, and the `robot_localization` node.
  - `ground_truth_bridge()` bridges Gazebo's `dynamic_pose/info` to `/ground_truth/poses`.
  - Nav2 and explore_lite start on `TimerAction`s (5 s and 20 s).
- **`tools/drift_report.py`**: reads a recorded bag and expresses ground truth, `/odom`, `/odometry/filtered`, and SLAM (`map→odom` ∘ `odom→base`) in the start frame. It reports position and heading error over time.
- **Config only**: `ekf_maze.yaml`, `slam_maze.yaml`, `nav2_maze.yaml`, `explore_maze.yaml`. These are parameter choices, not code; the comments in each file explain them.

## Loose ends a careful reader may notice

- The comments for `EXIT_CANDIDATE` in `frontier_memory.h:31` ("tried after open ones") and `Frontier.msg` predate `3dd8d3e`. Candidates are now chosen in the same order as open frontiers. The behaviour is correct; the comments are stale.
- The new explore_lite gtests aren't run in CI.
- `exit_watch_distance` (0.6 m) is set just above the closest Nav2 gets to some exits (about 0.55 m). It is tuned for this maze.
- `lidar_short_is_no_return` must be checked on the real LDS-01 before running on the robot.
