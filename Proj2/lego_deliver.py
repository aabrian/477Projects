from ultralytics import YOLO
import cv2
import numpy as np
import time
import imutils
from robomaster import robot
from robomaster import camera
import zmq


context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

if __name__ == '__main__':
    model = YOLO("Project2-5\\runs\detect\\train12\weights\\best.pt")

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKCH8800100TY")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    l_old = [1,1,2,1]
    counter = 5
    n = 0
    x_out,y_out,z_out = 0,0,0


    ################## PARAMETERS TO CHANGE DEPENDING ON LIGHT #####################
    kernel = (15,15)

    # river
    lowb = (100, 75, 61)
    highb = (145,255,255)
    ep_arm.moveto(x=180, y=40).wait_for_completed() # STARTING POSITION OF GRIPPER
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()
    x_goal = 60

    # goal
    lowo = (0,186,165)
    higho = (19,255,255)
    x_goal_end = 50

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
        Kx_riv = .0075 
        error_fb_end = x_goal_end - h_bigo
        Kx_end = .01

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
            message = str(socket.recv())
            if message == "arrived":
                counter = 2
        elif counter == 2: # center on other robot 
            FLAG =True
            while FLAG:
                frame = ep_camera.read_cv2_image(strategy="newest", timeout=2.5)
                frame_center = (int(frame.shape[1]/2),int(frame.shape[0]/2))
                if frame is not None:
                    if model.predictor:
                        model.predictor.args.verbose = False
                    results = model.predict(source=frame, show=True,half=True)
                    boxes = results[0].boxes
                    for box in results[0].boxes:
                        # print(results[0].names[int(box.cls.cpu().numpy())],box.cls,box.xyxy)
                        print(results[0].names[int(box.cls.cpu().numpy())])
                        # list.append(results[0].names[int(box.cls.cpu().numpy())])
                        if 'robot' in results[0].names[int(box.cls.cpu().numpy())]:
                            # print('sees lego')
                            #box = boxes[0].xyxy  # returns one box
                            box = box.xyxy
                            lego_center_x = ((box[0,0]+box[0,2])/2).item()
                            lego_center_y = ((box[0,1]+box[0,3])/2).item()
                            print(lego_center_x-frame_center[0])
                            # print(frame_center[0])
                            if n == 0:
                                if (int(lego_center_x) - frame_center[0]) > 0:
                                    ep_chassis.drive_speed(x = 0, y = .05, z = 0, timeout=10)
                                if (int(lego_center_x) - frame_center[0]) < 0:
                                    ep_chassis.drive_speed(x = 0, y = -.05, z = 0, timeout=10)
                                if abs(lego_center_x - frame_center[0]) < 15:
                                    n = 1
                                    # ep_chassis.drive_speed(x = 0.05, y = 0, z = 0, timeout=10)
                                    # if lego_center_x > 300 and lego_center_x<342.0 and lego_center_y>200:      
                                    #     ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=5)
                                    counter = 3
                                    print("Approach River")
                                    FLAG = False
            cv2.destroyWindow("image0.jpg")
        
        # Approaching river
        elif counter == 3:
            if abs(error_fb_riv) > 5:
                x_out = Kx_riv*error_fb_riv
                print(x_out)
                ep_chassis.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
            else:
                ep_chassis.drive_speed(x = .1, y = 0, z = 0, timeout=10)
                time.sleep(2.5)
                x_out = 0
                ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                counter = 4

        elif counter == 4: # send communication handoff is ready
          ep_gripper.close(power=100)
          time.sleep(1)
          ep_gripper.pause()  
          message = "True"
          socket.send(message)
          counter = 5
        elif counter == 5: 
          message = str(socket.recv())
          if message == "open":
            counter = 6

        elif counter == 6:
            ep_chassis.drive_speed(x = -.15, y = 0, z = 0, timeout=5)
            time.sleep(2)
            counter = 7
        elif counter == 7:
            ep_chassis.drive_speed(x = 0, y = 0, z = 7.5, timeout=5)
            if abs(centero[0] - frame_center[0]) < 5:
                counter = 8
                ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
        elif counter == 8:
            if abs(error_fb_end) > 5:
                x_out = Kx_end*error_fb_end
                ep_chassis.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
            else:
                ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                time.sleep(1)
                counter = 9
        elif counter == 9:
            ep_chassis.drive_speed(x = .15, y = 0, z = 0, timeout=10)
            time.sleep(2.75)
            ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
            counter = 10
        elif counter == 10:
            ep_arm.moveto(x=180, y=-30).wait_for_completed()
            ep_gripper.open(power=50)
            time.sleep(1)
            ep_gripper.pause()
            counter = 11
        elif counter == 11:
            ep_chassis.drive_speed(x = -.15, y = 0, z = 0, timeout=5)
            time.sleep(2)
            counter = 12
            ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=5)
        elif counter == 12:
            ep_chassis.drive_speed(x = 0, y = 0, z = 20, timeout=10)

        cv2.rectangle(frame, (x_bigb, y_bigb), (x_bigb + w_bigb, y_bigb + h_bigb), (255,0,0), 4)
        cv2.circle(frame, centerb, 5, (255, 0, 0), -1)
        cv2.rectangle(frame, (x_bigo, y_bigo), (x_bigo + w_bigo, y_bigo + h_bigo), (0,165,255), 4)
        cv2.circle(frame, centero, 5, (0, 165, 255), -1)
        cv2.imshow("bounding",frame)
        cv2.waitKey(10)