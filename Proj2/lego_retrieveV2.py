from ultralytics import YOLO
import cv2
import numpy as np
import time
import imutils
from robomaster import robot
from robomaster import camera

def pickup():
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()
    

    ep_gripper.close(power=100)
    time.sleep(1)
    ep_gripper.pause()
    
    ep_arm.moveto(x=170, y=-0).wait_for_completed()

if __name__ == '__main__':
    model = YOLO("/Users/david/RoboMaster-SDK/Project1/shared/Project2-5/runs/detect/train12/weights/last.pt")


    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="sta", sn="3JKCH8800100TY")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    l_old = [1, 1, 2, 1]
    counter = 0
    n = 0
    x_out,y_out,z_out = 0,0,0

    ################## PARAMETERS TO CHANGE DEPENDING ON LIGHT #####################
    kernel = (15,15)

    # low = (71,127, 140)
    # high = (180,255,255)

    low = (100, 75, 61)
    high = (145,255,255)
    ep_arm.moveto(x=180, y=-70).wait_for_completed()
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()
    x_goal = 70

    ################################################################################

    while True:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=2.5)
        frame_center = (int(frame.shape[1]/2),int(frame.shape[0]/2))
        
        
        if counter == 0:
            print("entering counter")
            FLAG = True
            while FLAG:
                # ret, frame = vid.read()
                frame = ep_camera.read_cv2_image(strategy="newest", timeout=2)
                if n ==0:
                    ep_chassis.drive_speed(x = 0, y = 0, z = 2, timeout=5)
                if frame is not None:
                    start = time.time()
                    if model.predictor:
                        model.predictor.args.verbose = False
                    results = model.predict(source=frame, show=True,half=True)
                    boxes = results[0].boxes
                    # print(results[0].names[])
                    # list = []
                    for box in results[0].boxes:
                    #     # print(results[0].names[int(box.cls.cpu().numpy())],box.cls,box.xyxy)
                        print(results[0].names[int(box.cls.cpu().numpy())])
                        # list.append(results[0].names[int(box.cls.cpu().numpy())])
                    
                        if 'lego' in results[0].names[int(box.cls.cpu().numpy())]:
                            # print('sees lego')
                            #box = boxes[0].xyxy  # returns one box
                            box = box.xyxy
                            lego_center_x = ((box[0,0]+box[0,2])/2).item()
                            lego_center_y = ((box[0,1]+box[0,3])/2).item()
                            if n == 0:
                                ep_chassis.drive_speed(x = 0, y = 0, z = -(int(lego_center_y) - frame_center[0])/40, timeout=2)
                                n = 1
                            if abs(int(lego_center_y) - frame_center[0]) < 30:
                                ep_chassis.drive_speed(x = 0, y = 0, z = 0)
                                print('stops')
                                n=2
                            if n==2:
                                ep_chassis.drive_speed(x = 0.05, y = 0, z = 0, timeout=10)
                            # ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=5)
                            # time.sleep(1)
                            ## close gripper when in range
                            # print('X=',lego_center_x)
                            # print('Y= ',lego_center_y)
                            if lego_center_x >300.0 and lego_center_x<342.0 and lego_center_y>195:      
                                # continue
                                # break
                                counter = 1
                                FLAG=False
            while True:
                frame = ep_camera.read_cv2_image(strategy="newest", timeout=2.5)
                frame_center = (int(frame.shape[1]/2),int(frame.shape[0]/2))
                if frame is not None:
                    if model.predictor:
                        model.predictor.args.verbose = False
                    results = model.predict(source=frame, show=True,half=True)
                    boxes = results[0].boxes
                    # ep_chassis.drive_speed(x = 0, y = 0, z = 7.5, timeout=5)
                    if len(boxes)>0:
                        box = boxes[0].xyxy  # returns one box
                        lego_center_x = ((box[0,0]+box[0,2])/2).item()
                        lego_center_y = ((box[0,1]+box[0,3])/2).item()
                        if n == 0:
                            ep_chassis.drive_speed(x = 0, y = 0, z = 3, timeout=5)
                            if abs(int(lego_center_y) - frame_center[0]) < 5:
                                n = 1
                        if n==1:
                            ep_chassis.drive_speed(x = 0.05, y = 0, z = 0, timeout=10)
                            # ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=5)
                            # time.sleep(1)
                        if lego_center_x >300.0 and lego_center_x<342.0 and lego_center_y>195:      
                            ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=5)
                            counter = 1
                            break
            pickup()
            ep_camera.stop_video_stream()
            cv2.destroyWindow("image0.jpg")


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
        error_rl = center[0] - frame_center[0]
        Ky = .01
        error_fb = x_goal - h_big
        Kx = .005

        if counter == 1:
            if linesP is not None:
                if abs(theta) > 0: # Get correct heading of robot
                    z_out = Kt*theta
                    ep_chassis.drive_speed(x = 0, y = 0, z = z_out, timeout=1)
                else:
                    ep_chassis.drive_speed(x = 0, y = 0, z = 2.25*z_out, timeout=1)
                    time.sleep(1)
                    z_out = 0
                    ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                    counter = 2
                    time.sleep(0.5)
            else:
                ep_chassis.drive_speed(x = 0, y = 0, z = 7.5, timeout=1)
        elif counter == 2: # approach river 
            if abs(error_fb) > 5:
                x_out = Kx*error_fb
                ep_chassis.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
            else:
                ep_chassis.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
                time.sleep(3)
                x_out = 0
                ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                counter == 3
            
        cv2.rectangle(frame, (x_big, y_big), (x_big + w_big, y_big + h_big), (0,255,255), 4)
        cv2.circle(frame, center, 5, (0, 255, 255), -1)

        ###################### INSERT COMMUNICATION AND HANDOFF CODE ##############################

        # add in counters
            # 1. wait for okay from other robot, begin handoff code
        # elif counter == 4: # send communication handoff is ready
        #     if communication recieved:
        #             ep_gripper.open(power=50)
        #             time.sleep(1)
        #             ep_gripper.pause()
        #             ep_chassis.drive_speed(x = -.25, y = 0, z = 0, timeout=1)
        #             time.sleep(2)
        #             ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)

        ###########################################################################################
        
        cv2.rectangle(frame, (x_big, y_big), (x_big + w_big, y_big + h_big), (0,255,255), 4)
        cv2.circle(frame, center, 5, (0, 255, 255), -1)
        cv2.imshow("bounding",frame)
        cv2.waitKey(10)