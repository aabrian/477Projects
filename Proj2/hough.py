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


while True:
    frame = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
    # ret, frame = vid.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    low = (71,130, 46)
    high = (180,255,255)
    mask = cv2.inRange(hsv, low, high)
    # mask = cv2.erode(mask, None, iterations=2)
    # mask = cv2.dilate(mask, None, iterations=2)

    thresh = 180
    thresh_frame = cv2.threshold(mask, thresh, 255, cv2.THRESH_BINARY)[1]
    blur = cv2.GaussianBlur(thresh_frame, (5, 5), 0)
    edge_frame = cv2.Canny(blur, 30, 150)

    lines = cv2.HoughLines(edge_frame, 1, np.pi / 180, 30)

    
    
    
    # cv2.imshow('masked', mask)
    # cv2.imshow('or frame', or_frame)
    # cv2.imshow('thresh', thresh_frame)
    cv2.imshow('edges', edge_frame)
    # cv2.imshow("Bounding Box",frame)
    cv2.waitKey(10)
