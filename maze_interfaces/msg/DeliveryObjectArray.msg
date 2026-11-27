# Every object of the mission and its state. Latched (transient local) so a node
# that starts late still gets the latest list.
std_msgs/Header header        # frame_id: map
DeliveryObject[] objects
