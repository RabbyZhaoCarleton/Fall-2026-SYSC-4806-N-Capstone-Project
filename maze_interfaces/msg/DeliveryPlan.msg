# The global delivery plan: the most time-efficient order in which to pick up
# the remaining objects and take them to their exits, from the robot's current pose.
# Published latched on /delivery/plan by the global_planner node, and returned
# by its get_plan service.

# status values
uint8 OK=0                    # every remaining object is in the plan
uint8 PARTIAL=1               # some objects could not be planned (see unplanned_*)
uint8 NOTHING_TO_DO=2         # no object is waiting or carried
uint8 NO_PLAN=3               # nothing could be planned (no costmap, no robot pose, ...)

std_msgs/Header header        # frame_id: map; stamp: when it was planned
uint32 plan_id                # increases with every new plan
uint8 status
string message                # human-readable summary or reason

string solver                 # "exact" (optimal) or "heuristic" (too many objects for exact)
uint32 capacity               # objects the robot can carry at once
float32 total_time            # s, estimated time of the whole plan
float32 total_distance        # m, estimated driving distance of the whole plan
float32 planning_time         # s the planner took (computation, not driving)

PlanStep[] steps

string[] unplanned_object_ids # objects left out of the plan
string[] unplanned_reasons    # why, same order
