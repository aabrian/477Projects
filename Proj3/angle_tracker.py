import cv2
import numpy as np
import time
import imutils
from robomaster import robot
from robomaster import camera
import math

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
ep_camera = ep_robot.camera
ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
ep_chassis1 = ep_robot.chassis
ep_arm = ep_robot.robotic_arm
ep_gripper = ep_robot.gripper

l_graph = 16 #side length of each box in csv

def add_wall(angle,height,width):
    ratio = height/width
    if ratio < 1: # single box
        rad = height*0.2
        x = rad*math.cos(angle)
        y = rad*math.sin(angle)
        x_wall = int((x_start + x)/l_graph)
        y_wall = int((y_start - y)/l_graph)
    elif ratio < 2 and ratio > 1: # double box
        
    else: # clump




if __name__ == '__main__':
    file = cv2.reader(open('map_left.csv'), delimiter=',')
    x = list(file)
    maze = np.array(x).astype("int")

    frame = ep_camera.read_cv2_image(strategy="newest", timeout=2.5)
    frame_center = (int(frame.shape[1]/2),int(frame.shape[0]/2))
    

    frame = cv2.putText(frame, str(angle), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 
                   1, (0,255,255), 2, cv2.LINE_AA)
    cv2.imshow(frame,"frame")
    cv2.waitKey(10)