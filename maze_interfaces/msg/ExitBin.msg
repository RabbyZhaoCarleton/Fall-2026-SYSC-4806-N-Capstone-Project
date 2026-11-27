# One maze exit with its collection bin.
#
# The position comes from explore_lite's exit detection (/explore/exits); the id
# comes from the bin's sign / QR code, read by the GCS. The task manager joins the
# two and publishes them (inside ExitBinArray).

string id                     # bin name, e.g. "exit_A" (DeliveryObject.exit_id refers to it)
geometry_msgs/Pose pose       # centre of the exit opening, in the header's frame (map)
