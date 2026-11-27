# One step of the global delivery plan. Steps are done in order.
#
# NAVIGATE steps are handed to Nav2 (NavigateToPose) by the task manager; Nav2's
# planner and controller (local planning) find and follow the actual route.
# PICK and PLACE steps are handed to the manipulation subsystem.

# action values
uint8 NAVIGATE_TO_PICKUP=0    # drive to the pose in front of object_id
uint8 PICK=1                  # pick up object_id
uint8 NAVIGATE_TO_EXIT=2      # drive to the drop-off pose in front of exit_id
uint8 PLACE=3                 # drop object_id into the bin of exit_id

uint8 action
uint32 trip                   # 0, 1, 2, ...: one trip = pick up (up to capacity), then deliver
string object_id              # object of PICK / PLACE / NAVIGATE_TO_PICKUP ("" otherwise)
string exit_id                # exit of NAVIGATE_TO_EXIT / PLACE ("" otherwise)
geometry_msgs/Pose target     # NAVIGATE_*: goal pose for Nav2 (map frame); PICK/PLACE: the robot's pose then
float32 distance              # m driven in this step (0 for PICK / PLACE)
float32 duration              # s, estimated time of this step
float32 start_time            # s after the plan starts, estimated
