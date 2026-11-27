# One object the robot has to deliver to an exit bin.
#
# Published (inside DeliveryObjectArray) by the task manager, which combines the
# GCS classification (class + assigned exit, proposal 2.3.3 step 4) with where
# the object is. The global planner only reads these.

# state values
uint8 WAITING=0     # at the entrance, not picked up yet
uint8 CARRIED=1     # in the gripper (picked up, not delivered yet)
uint8 DELIVERED=2   # dropped at its exit bin; the planner ignores it
uint8 FAILED=3      # given up on (for example 5 failed pickups); the planner ignores it

string id                     # unique name, e.g. "obj_1"
string object_class           # class from the GCS classifier, e.g. "red_cube"
string exit_id                # exit bin this object goes to (matches ExitBin.id)
geometry_msgs/Pose pose       # where the object is, in the header's frame (map)
uint8 state
