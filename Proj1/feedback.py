from pupil_apriltags import Detector
from time import sleep
import cv2
import numpy as np
import time



# time_step = 5
# t = 0
# final_range = 0.05
# while True:
#     if curr_coord()[0] > -1*final_range  and curr_coord()[0] < final_range:
#         if curr_coord()[1] > -1*final_range  and curr_coord()[1] < final_range:
#             ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
#             break

#     Feedback
#     K = 1
#     x_pos_des = interp(t)[0]
#     y_pos_des = interp(t)[1]
#     curr_x = curr_coord()[0]
#     curr_y = curr_coord()[1]

#     Feedfoward
#     x_vel_des = derivative(t)[0]
#     y_vel_des = derivative(t)[1]

#     Control Law
#     output_x = K*(x_pos_des - curr_x) + x_vel_des
#     output_y = K*(y_pos_des - curr_y) + y_vel_des

#     Movement
#     ep_chassis.drive_speed(x = output_x, y = output_y, z=0, timeout=1)
#     t = t + time_step