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
vid = cv2.VideoCapture(0)


# ep_robot = robot.Robot()
# ep_robot.initialize(conn_type="ap")
# ep_camera = ep_robot.camera
# ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

l_old = [1, 1, 2, 1]

################## PARAMETERS TO CHANGE DEPENDING ON LIGHT #####################
kernel = (21,21)
low = (71,127, 140)
high = (180,255,255)
# low = (71,130, 46)
# high = (180,255,255)
goal_height = 30 # For x translation



################################################################################
while True:
    # frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
    ret, frame = vid.read()
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
    for i in cnts:
        x,y,w,h = cv2.boundingRect(i)
        if w > w_big and h > h_big:
            w_big = w
            h_big = h
            x_big = x
            y_big = y
    center = (int(x_big + (w_big/2)),int(y_big + (h_big/2)))


    linesP = cv2.HoughLinesP(edge_frame, 1, np.pi / 180, 50, None, 50, 10)

    if linesP is not None:
        if len(linesP) == 2:
            if (linesP[1][0][1]+linesP[1][0][3])/2 > (linesP[0][0][1]+linesP[0][0][3])/2:
                l = linesP[0][0]
                cv2.line(frame, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
                l_old = l
            else:
                l = linesP[1][0]
                cv2.line(frame, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv2.LINE_AA)
                l_old = l
        else:
            cv2.line(frame, (l_old[0], l_old[1]), (l_old[2], l_old[3]), (0,0,255), 3, cv2.LINE_AA)
    else:
        cv2.line(frame, (l_old[0], l_old[1]), (l_old[2], l_old[3]), (0,0,255), 3, cv2.LINE_AA)

    diff_left = l_old[1] - y_big
    diff_right = l_old[3] - y_big
    theta = (diff_right-diff_left)/(l_old[2]-l_old[0])
    Kt = 5
    if abs(theta) > 0:
        z_out = Kt*theta
    else:
        z_out = 0
    
    error_rl = center[0] - frame_center[0]
    Ky = .01
    if abs(error_rl) > 0:
       y_out = Ky*error_rl
    else:
       y_out = 0
        
    cv2.rectangle(frame, (x_big, y_big), (x_big + w_big, y_big + h_big), (0,255,255), 4)
    # cv2.line(frame, (x_big, y_big), (x_big + w_big, y_big + h_big), (0,0,255), 3, cv2.LINE_AA)
    cv2.circle(frame, center, 5, (0, 255, 255), -1)
    
    cv2.imshow('masked', mask)
    cv2.imshow('or frame', blur)
    cv2.imshow('thresh', thresh_frame)
    cv2.imshow('edges', edge_frame)
    cv2.imshow("bounding",frame)
    cv2.waitKey(10)

# while True:
#    if no bounding box, then rotate in z direction
#    find bounding box coordinates
#    find hough line coordinate of top line
#       get theta comparing these coordinates, can add rotation
#       controller in y direction using center of bounding box
#       controller in x direction using height of bounding box
