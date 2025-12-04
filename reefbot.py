from MiniTags import MiniTags
from Communicator import Communicator
from math import atan2
import time
import numpy as np


fx = (2.75 / 6.45) * 480
fy = (2.75 / 3.63) * 240
oops = np.array([[fx, 0, 240],
                 [0, fy, 120],
                 [0, 0, 1]])
minitags = MiniTags(camera_matrix=oops)
# My camera produces minimal distortion and I forgot to calibrate it at the final resolution, so I am going to use a
# guess for the camera matrix
# minitags.calibrate()   # Loads/Creates camera parameters to correct for lens distortion
uart = Communicator()  # Allows data to be sent to the ESP32 over UART

# Init heartbeat with tag -1
uart.send("-1,0,0,0")
last_tag = time.time()

while True:
    closest_tag = None
    closest_distance = 9999

    for tag in minitags.get_tags():
        trans = tag.pose_t
        x = trans[0].item()
        z = trans[2].item()
        length = x**2 + z**2
        if length > closest_distance: continue

        closest_tag = tag
        closest_distance = length

    if closest_tag:
        t = closest_tag.pose_t
        x = t[0].item()
        z = t[2].item()
        rot = atan2(x, z)
        uart.send("%s,%s,%s,%s" % (closest_tag.tag_id, x, z, rot))
        last_tag = time.time()
    elif last_tag + 3 < time.time():
        uart.send("-1,0,0,0")
        last_tag = time.time()
