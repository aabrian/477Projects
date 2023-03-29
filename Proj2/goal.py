from pupil_apriltags import Detector
import cv2
import csv
import numpy as np
import time
import csv
import imutils
from robomaster import robot
from robomaster import camera
import math


# Use vid instead of ep_camera to use your laptop's webcam
# vid = cv2.VideoCapture(0)


ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
ep_camera = ep_robot.camera
ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
ep_chassis = ep_robot.chassis
ep_gripper = ep_robot.gripper
ep_arm = ep_robot.robotic_arm

l_old = [1, 1, 2, 1]
counter = 0
x_out,y_out,z_out = 0,0,0

################## PARAMETERS TO CHANGE DEPENDING ON LIGHT #####################
kernel = (15,15)

low = (0,186,107)
high = (19,255,255)

x_goal_end = 50
################################################################################

while True:
    frame = ep_camera.read_cv2_image(strategy="newest", timeout=2.5)
    # ret, frame = vid.read()
    frame_center = (int(frame.shape[1]/2),int(frame.shape[0]/2))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    mask_bound = cv2.erode(mask, None, iterations=2)
    mask_bound = cv2.dilate(mask, None, iterations=2)
    thresh = 180
    thresh_frame = cv2.threshold(mask, thresh, 255, cv2.THRESH_BINARY)[1]
    blur = cv2.GaussianBlur(thresh_frame, kernel, 0) 
    edge_frame = cv2.Canny(blur, 30, 150)
    cnts = cv2.findContours(mask_bound, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    w_big = 0
    h_big = 0
    x_big = 0
    y_big = 0
    if cnts is not None:
        for i in cnts:
            x,y,w,h = cv2.boundingRect(i)
            if h/w < 1:
                if w > w_big and h > h_big:
                    w_big = w
                    h_big = h
                    x_big = x
                    y_big = y
        center = (int(x_big + (w_big/2)),int(y_big + (h_big/2)))

    error_fb_end = x_goal_end - h_big
    Kx = .01
    if counter == 0:
        ep_chassis.drive_speed(x = -.15, y = 0, z = 0, timeout=5)
        time.sleep(2)
        counter = 1
    elif counter == 1:
        ep_chassis.drive_speed(x = 0, y = 0, z = 7.5, timeout=5)
        if abs(center[0] - frame_center[0]) < 5:
            counter = 2
            ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
    elif counter == 2:
        if abs(error_fb_end) > 5:
            x_out = Kx*error_fb_end
            ep_chassis.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
        else:
            ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
            time.sleep(1)
            counter = 3
    elif counter == 3:
        ep_chassis.drive_speed(x = .15, y = 0, z = 0, timeout=10)
        time.sleep(2.75)
        ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
        counter = 4
    elif counter == 4:
        ep_gripper.open(power=50)
        time.sleep(1)
        ep_gripper.pause()
        print("Lego dropped")
        counter = 5


    cv2.rectangle(frame, (x_big, y_big), (x_big + w_big, y_big + h_big), (0,255,255), 4)
    cv2.circle(frame, center, 5, (0, 255, 255), -1)
    
    # cv2.imshow('masked', mask)
    # cv2.imshow('blurred', blur)
    # cv2.imshow('thresh', thresh_frame)
    # cv2.imshow('edges', edge_frame)
    cv2.imshow("bounding",frame)
    cv2.waitKey(10)
