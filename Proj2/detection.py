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


# ep_robot = robot.Robot()
# ep_robot.initialize(conn_type="ap")
# ep_camera = ep_robot.camera
# ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)


while True:
    ret, frame = vid.read()
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(frame, (112, 0, 20), (255, 145, 33))
    #cv2.imshow("frame", frame)
    #cv2.imshow("hsv_image",mask)
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

    cv2.rectangle(frame, (x_big, y_big), (x_big + w_big, y_big + h_big), (0,0,255), 4)
    cv2.imshow("Bounding Box",frame)
    cv2.waitKey(10)
