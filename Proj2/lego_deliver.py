import cv2
import numpy as np
import time
import imutils
from robomaster import robot
from robomaster import camera

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm

    counter = 0
    x_out,y_out,z_out = 0,0,0

    ################## PARAMETERS TO CHANGE DEPENDING ON LIGHT #####################
    kernel = (15,15)

    # river
    low = (100, 75, 61)
    high = (145,255,255)
    ep_arm.moveto(x=100, y=40).wait_for_completed() # STARTING POSITION OF GRIPPER
    x_goal = 60

    # goal

    ################################################################################


    while True:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=2.5)
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
        
        linesP = cv2.HoughLinesP(edge_frame, 1, np.pi / 180, 50, None, 50, 10)
        
        if linesP is not None:
            if len(linesP) > 2:
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
        Kt = 50

        if counter == 0:
            if abs(theta) > 0:
                z_out = Kt*theta
                ep_chassis.drive_speed(x = 0, y = 0, z = z_out, timeout=1)
            else:
                ep_chassis.drive_speed(x = 0, y = 0, z = 2.25*z_out, timeout=1)
                time.sleep(1)
                z_out = 0
                ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                counter = 1
                time.sleep(0.5)
        
        ###################### INSERT COMMUNICATION CODE ##############################

        # add in counters
            # 1. wait for okay from first robot, flip counter
            # 2. Center on other robot using machine learning, flip counter

        ################################################################################
        
        error_fb = x_goal - h_big
        Kx = .005 
        if counter == 2:
            if abs(error_fb) > 5:
                x_out = Kx*error_fb
                ep_chassis.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
            else:
                ep_chassis.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
                time.sleep(3)
                x_out = 0
                ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                counter == 3


        ###################### INSERT COMMUNICATION and HANDOFF CODE ###################

        # add in counters
            # 1. send okay to other robot, begin handoff code

        ################################################################################


        ################################## INSERT GOAL CODE ############################



        # add dropping code
        ################################################################################