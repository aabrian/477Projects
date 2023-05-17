from robomaster import robot
from matplotlib import pyplot
import time
import threading
import numpy as np
from scipy import interpolate
import csv
import pandas as pd
import math
import cv2

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
ep_sensor = ep_robot.sensor
ep_chassis = ep_robot.chassis

def sub_position_handler(p, x_new):
    x_new[0] = p[0]
    x_new[1] = p[1]
    x_new[2] = p[2]
    # print("chassis position: x: {}".format(x_new))

wait_to_start_moving = True
def move_square(ep_chassis, x_len=.75, y_len=.75, speed=1.0):
    while True:
        ep_chassis.move(x=x_len,  y=0,      z=0, xy_speed=speed).wait_for_completed()
        ep_chassis.move(x=0,      y=y_len,  z=0, xy_speed=speed).wait_for_completed()
        ep_chassis.move(x=-x_len, y=0,      z=0, xy_speed=speed).wait_for_completed()
        ep_chassis.move(x=0,      y=-y_len, z=0, xy_speed=speed).wait_for_completed()

if __name__ == '__main__':
    # Wheel odometry position estimates for rescaling visual odometry t vector
    x_list = []
    x_old = np.zeros((3,))
    x_new = np.zeros((3,))
    frame_undistorted_gray_old = None

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_robot.chassis.sub_position(freq=50, callback=lambda p: sub_position_handler(p, x_new))
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)

    x = threading.Thread(target=move_square, daemon=True, args=(ep_robot.chassis,))
    x.start()

    # For visualization
    trajectory_plot = np.zeros((480, 640, 3), dtype=np.uint8)
    plot_off_x = int(trajectory_plot.shape[1]/2)
    plot_off_y = int(trajectory_plot.shape[0]/2)
    # Each pixel in the plot is 1 meter / plot_scale
    plot_scale = 100

    while True:
        if x_old is None:
            x_old = np.copy(x_new)
        
        cv2.circle(trajectory_plot,
            (int(plot_scale * x_new[1] + plot_off_x),
            int(plot_scale * x_new[0] + plot_off_y)),
            1, (0,0,255), 1)

        cv2.imshow('Trajectory plot', trajectory_plot)
        cv2.waitKey(1)

        x_old = np.copy(x_new)