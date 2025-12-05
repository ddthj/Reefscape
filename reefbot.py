from MiniTags import MiniTags
from Communicator import Communicator
from math import atan2
import time
import numpy as np


# My camera produces minimal distortion and I forgot to calibrate it at the final resolution, so I am going to use a
# guess for the camera matrix

fx = 310
fy = 310
oops_matrix = np.array([[fx, 0, 240],
                 [0, fy, 120],
                 [0, 0, 1]])
oops_distortion = np.array([[-0.038835, 0.10040667, -0.0019115, 0.00218258, -0.05835169]])


minitags = MiniTags(camera_matrix=oops_matrix, camera_distortion=oops_distortion)
uart = Communicator()  # Allows data to be sent to the ESP32 over UART

# Init heartbeat with tag -1
uart.send("-1,0,0,0")
last_tag = time.time()

while True:
    closest_tag = None
    closest_distance = 99999

    for tag in minitags.get_tags():
        trans = tag.pose_t
        x = trans[0].item()
        z = trans[2].item()
        length = x**2 + z**2
        if length > closest_distance: continue

        closest_tag = tag
        closest_distance = length

    if closest_tag:
        r = closest_tag.pose_R.T
        t = -np.dot(r, closest_tag.pose_t)
        x = t[0].item()
        z = t[2].item()
        rot = -atan2(r[0, 2], r[2, 2])
        uart.send("%s,%s,%s,%s" % (closest_tag.tag_id, x, z, rot))
        last_tag = time.time()
    elif last_tag + 3 < time.time():
        uart.send("-1,0,0,0")
        last_tag = time.time()
