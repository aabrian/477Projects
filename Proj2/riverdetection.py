from pupil_apriltags import Detector
import cv2
import csv
import numpy as np
import time
import csv
import imutils
from robomaster import robot
from robomaster import camera


# Use vid instead of ep_camera to use your laptop's webcam
vid = cv2.VideoCapture(0)


ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
ep_camera = ep_robot.camera
ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
ep_chassis = ep_robot.chassis

count = 0
while True:
    frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
    # ret, frame = vid.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low = (71,130, 46)
    high = (180,255,255)
    mask = cv2.inRange(hsv, low, high)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
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
    print(h_big,w_big/h_big)
    cv2.rectangle(frame, (x_big, y_big), (x_big + w_big, y_big + h_big), (0,0,255), 4)

    if count == 0:
        Kz = .3
        out_z = Kz*(h_big-11)
        output = 0
    
    if w_big/h_big >= 8:
        count = 1
    
    if count == 1:
        K = .05
        output = K*(30-h_big)
        out_z = 0
    ep_chassis.drive_speed(x = output, y = 0, z= out_z, timeout=1)
    cv2.imshow("Bounding Box",frame)
    cv2.waitKey(10)
