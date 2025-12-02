from MiniTags import MiniTags
from Communicator import Communicator
from math import atan2

# Example usage
minitags = MiniTags()
minitags.calibrate()   # Loads/Creates camera parameters to correct for lens distortion
uart = Communicator()  # Allows data to be sent to the ESP32 over UART

# Init heartbeat with tag -1
uart.send("-1,0,0,0")

while True:
    closest_tag = None
    closest_distance = 9999

    for tag in minitags.get_tags():
        trans = tag.pose_t
        length = trans[0]**2 + trans[2]**2
        if length > closest_distance: continue

        closest_tag = tag
        closest_distance = length

    if closest_tag:
        t = closest_tag.pose_t
        rot = atan2(t[0], t[2])
        uart.send("%s,%s,%s,%s" % (closest_tag.tag_id, t[0], t[2], rot))
