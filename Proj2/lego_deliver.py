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
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    counter = 0
    x_out,y_out,z_out = 0,0,0


    ################## PARAMETERS TO CHANGE DEPENDING ON LIGHT #####################
    kernel = (15,15)

    # river
    lowb = (100, 75, 61)
    highb = (145,255,255)
    ep_arm.moveto(x=100, y=40).wait_for_completed() # STARTING POSITION OF GRIPPER
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()
    x_goal = 60

    # goal
    lowo = (0,186,107)
    higho = (19,255,255)
    x_goal_end = 50

    # lego
    lowl = (0,186,107)
    highl = (19,255,255)

    ################################################################################



    while True:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=2.5)
        frame_center = (int(frame.shape[1]/2),int(frame.shape[0]/2))

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # BLUE RIVER DETECTION
        maskb = cv2.inRange(hsv, lowb, highb)
        mask_boundb = cv2.erode(maskb, None, iterations=2)
        mask_boundb = cv2.dilate(maskb, None, iterations=2)
        thresh = 180
        thresh_frameb = cv2.threshold(maskb, thresh, 255, cv2.THRESH_BINARY)[1]
        blurb = cv2.GaussianBlur(thresh_frameb, kernel, 0) 
        edge_frameb = cv2.Canny(blurb, 30, 150)
        cntsb = cv2.findContours(mask_boundb, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cntsb = imutils.grab_contours(cntsb)
        w_bigb,h_bigb,x_bigb,y_bigb = 0,0,0,0
        for i in cntsb:
            xb,yb,wb,hb = cv2.boundingRect(i)
            if wb > w_bigb and hb > h_bigb:
                w_bigb = wb
                h_bigb = hb
                x_bigb = xb
                y_bigb = yb
            centerb = (int(x_bigb + (w_bigb/2)),int(y_bigb + (h_bigb/2)))
        linesP = cv2.HoughLinesP(edge_frameb, 1, np.pi / 180, 50, None, 50, 10)
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

        # LEGO DETECTION
        maskl = cv2.inRange(hsv, lowl, highl)
        mask_boundl = cv2.erode(maskl, None, iterations=2)
        mask_boundl = cv2.dilate(maskl, None, iterations=2)
        thresh_framel = cv2.threshold(mask_boundl, thresh, 255, cv2.THRESH_BINARY)[1]
        blurl = cv2.GaussianBlur(thresh_framel, kernel, 0) 
        edge_framel = cv2.Canny(blurl, 30, 150)
        cntsl = cv2.findContours(mask_boundl, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cntsl = imutils.grab_contours(cntsl)
        w_bigl,h_bigl,x_bigl,y_bigl = 0,0,0,0
        if cntsl is not None:
            for i in cntsl:
                xl,yl,wl,hl = cv2.boundingRect(i)
                if hl/wl < 1:
                    if wl > w_bigl and hl > h_bigl:
                        w_bigo = wl
                        h_bigo = hl
                        x_bigo = xl
                        y_bigo = yl
            centerl = (int(x_bigl + (w_bigl/2)),int(y_bigl + (h_bigl/2)))

        # GOAL DETECTION
        masko = cv2.inRange(hsv, lowo, higho)
        mask_boundo = cv2.erode(masko, None, iterations=2)
        mask_boundo = cv2.dilate(masko, None, iterations=2)
        thresh_frameo = cv2.threshold(masko, thresh, 255, cv2.THRESH_BINARY)[1]
        bluro = cv2.GaussianBlur(thresh_frameo, kernel, 0) 
        edge_frameo = cv2.Canny(bluro, 30, 150)
        cntso = cv2.findContours(mask_boundo, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cntso = imutils.grab_contours(cntso)
        w_bigo,h_bigo,x_bigo,y_bigo = 0,0,0,0
        if cntso is not None:
            for i in cntso:
                xo,yo,wo,ho = cv2.boundingRect(i)
                if ho/wo < 1:
                    if wo > w_bigo and ho > h_bigo:
                        w_bigo = wo
                        h_bigo = ho
                        x_bigo = xo
                        y_bigo = yo
            centero = (int(x_bigo + (w_bigo/2)),int(y_bigo + (h_bigo/2)))
        

        # error calculations
        diff_left = l_old[1] - y_bigb
        diff_right = l_old[3] - y_bigb
        theta = (diff_right-diff_left)/(l_old[2]-l_old[0]) # Controller to rotate theta by
        Kt = 50
        error_fb_riv = x_goal - h_bigb
        Kx_riv = .005 
        error_fb_end = x_goal_end - h_bigo
        Kx_end = .01
        error_rl = frame_center[0] - centerl[0]
        Ky = .01

        if counter == 0: # rotating towards river
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

        elif counter == 1: # wait for communication
          #if communication recieved:
              counter = 2
        elif counter == 2: # center on other robot 
            if abs(error_rl) > 5:
                y_out = Ky*error_rl
                ep_chassis.drive_speed(x = 0, y = y_out, z = 0, timeout=1)
            else:
                # ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1) # any adjustment needed
                # time.sleep(3)
                y_out = 0
                ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                counter = 3
        
        # Approaching river
        elif counter == 3:
            if abs(error_fb_riv) > 5:
                x_out = Kx_riv*error_fb_riv
                ep_chassis.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
            else:
                ep_chassis.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
                time.sleep(3)
                x_out = 0
                ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                counter = 4


        ###################### INSERT COMMUNICATION and HANDOFF CODE ###################

        # elif counter == 4: # send communication handoff is ready
        #   ep_gripper.close(power=100)
        #   time.sleep(1)
        #   ep_gripper.pause()  
        #   send communication
        #   time.sleep(3)
        #   counter = 5

        ################################################################################

        elif counter == 5:
            ep_chassis.drive_speed(x = -.15, y = 0, z = 0, timeout=5)
            time.sleep(2)
            counter = 6
        elif counter == 6:
            ep_chassis.drive_speed(x = 0, y = 0, z = 7.5, timeout=5)
            if abs(centero[0] - frame_center[0]) < 5:
                counter = 7
                ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
        elif counter == 7:
            if abs(error_fb_end) > 5:
                x_out = Kx_end*error_fb_end
                ep_chassis.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
            else:
                ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                time.sleep(1)
                counter = 8
        elif counter == 8:
            ep_chassis.drive_speed(x = .15, y = 0, z = 0, timeout=10)
            time.sleep(2.75)
            ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
            counter = 9
        elif counter == 9:
            ep_arm.moveto(x=180, y=-30).wait_for_completed()
            ep_gripper.open(power=50)
            time.sleep(1)
            ep_gripper.pause()
            counter = 10
        elif counter == 10:
            ep_chassis.drive_speed(x = -.15, y = 0, z = 0, timeout=5)
            time.sleep(2)
            counter = 11
            ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=5)

        cv2.rectangle(frame, (x_bigb, y_bigb), (x_bigb + w_bigb, y_bigb + h_bigb), (255,0,0), 4)
        cv2.circle(frame, centerb, 5, (255, 0, 0), -1)
        cv2.rectangle(frame, (x_bigo, y_bigo), (x_bigo + w_bigo, y_bigo + h_bigo), (0,165,255), 4)
        cv2.circle(frame, centero, 5, (0, 165, 255), -1)
        cv2.rectangle(frame, (x_bigl, y_bigl), (x_bigl + w_bigl, y_bigl + h_bigl), (0,165,255), 4)
        cv2.circle(frame, centerl, 5, (0, 255, 255), -1)  
        cv2.imshow("bounding",frame)
        cv2.waitKey(10)