from ultralytics import YOLO
import cv2
import numpy as np
import time
import imutils
from robomaster import robot
from robomaster import camera
import zmq

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.50.134:5555") # figure out IP adress stuff

def pickup():
    ep_gripper1.open(power=50)
    time.sleep(1)
    ep_gripper1.pause()
    

    ep_gripper1.close(power=100)
    time.sleep(1)
    ep_gripper1.pause()
    
    ep_arm1.moveto(x=170, y=-0).wait_for_completed()

def send_gripper_status(sub_info):
    time.sleep(3)
    global gripper_status
    gripper_status = sub_info
    global message
    message = gripper_status
    socket.send_string(message)
    print(message)

def Robot_destiniation(counter):
    time.sleep(3)
    if counter == 3:
        message = "arrived"
        socket.send_string(message)

if __name__ == '__main__':
    model = YOLO("Project2-5\\runs\detect\\train12\weights\\best.pt")


    ep_robot1 = robot.Robot()
    ep_robot1.initialize(conn_type="sta",sn = "3JKCH8800100WV")
    ep_camera1 = ep_robot1.camera
    ep_camera1.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis1 = ep_robot1.chassis
    ep_arm1 = ep_robot1.robotic_arm
    ep_gripper1 = ep_robot1.gripper
    ep_arm1 = ep_robot1.robotic_arm

    l_old = [1, 1, 2, 1]
    counter1 = 0
    n1 = 0
    x_out,y_out,z_out = 0,0,0

    ################## PARAMETERS TO CHANGE DEPENDING ON LIGHT #####################
    kernel = (15,15)

    # low = (71,127, 140)
    # high = (180,255,255)

    low = (100, 75, 61)
    high = (145,255,255)
    ep_arm1.moveto(x=180, y=-70).wait_for_completed()
    ep_gripper1.open(power=50)
    time.sleep(1)
    ep_gripper1.pause()
    x_goal = 70
    ######################### PARAMETERS FOR THE DELIVER ROBOT
    ep_robot2 = robot.Robot()
    # ep_robot2.initialize(conn_type="ap")
    ep_robot2.initialize(conn_type="sta", sn="3JKCH8800100TY")
    ep_camera2 = ep_robot2.camera
    ep_camera2.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis2 = ep_robot2.chassis
    ep_arm2 = ep_robot2.robotic_arm
    ep_gripper2 = ep_robot2.gripper
    ep_arm2 = ep_robot2.robotic_arm

    l_old = [1,1,2,1]
    counter2 = -1
    n2 = 0
    x_out,y_out,z_out = 0,0,0


    ################## PARAMETERS TO CHANGE DEPENDING ON LIGHT #####################
    kernel = (15,15)

    # river
    lowb = (100, 75, 61)
    highb = (145,255,255)
    ep_arm2.moveto(x=180, y=40).wait_for_completed() # STARTING POSITION OF GRIPPER
    ep_gripper2.open(power=50)
    time.sleep(1)
    ep_gripper2.pause()
    x_goal = 60

    # goal
    lowo = (0,186,165)
    higho = (19,255,255)
    x_goal_end = 50
    cameraFLag = 0
    ################################################################################    
    ################################################################################

    while True:
        if cameraFLag == 0:
            frame = ep_camera1.read_cv2_image(strategy="newest", timeout=2.5)
            frame_center = (int(frame.shape[1]/2),int(frame.shape[0]/2))
        
        
        if counter1 == 0:
            FLAG =True
            while FLAG:
                frame = ep_camera1.read_cv2_image(strategy="newest", timeout=2.5)
                frame_center = (int(frame.shape[1]/2),int(frame.shape[0]/2))
                if frame is not None:
                    if model.predictor:
                        model.predictor.args.verbose = False
                    results = model.predict(source=frame, show=True,half=True)
                    boxes = results[0].boxes
                    if n1 == 0:
                        ep_chassis1.drive_speed(x = 0, y = 0, z = 5, timeout=10)
                    for box in results[0].boxes:
                        # print(results[0].names[int(box.cls.cpu().numpy())],box.cls,box.xyxy)
                        print(results[0].names[int(box.cls.cpu().numpy())])
                        # list.append(results[0].names[int(box.cls.cpu().numpy())])
                        if 'lego' in results[0].names[int(box.cls.cpu().numpy())]:
                            # print('sees lego')
                            #box = boxes[0].xyxy  # returns one box
                            box = box.xyxy
                            lego_center_x = ((box[0,0]+box[0,2])/2).item()
                            lego_center_y = ((box[0,1]+box[0,3])/2).item()
                            print(lego_center_y)
                            # print(frame_center[0])
                            if n1 == 0:
                                ep_chassis1.drive_speed(x = 0, y = 0, z = 4, timeout=10)
                                if abs(int(lego_center_x) - frame_center[0]) < 25:
                                    n1 = 1
                                    ep_chassis1.drive_speed(x = 0.1, y = 0, z = 0, timeout=10)
                            if lego_center_x >300.0 and lego_center_x<342.0 and lego_center_y>130:      
                                ep_chassis1.drive_speed(x = 0, y = 0, z = 0, timeout=5)
                                counter1 = 1
                                FLAG = False
            pickup()
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

        if counter1 == 1:
            print("looking for river")
            if linesP is not None:
                print(theta)
                if abs(theta) > 0: # Get correct heading of robot
                    print(z_out)
                    z_out = Kt*theta
                    ep_chassis1.drive_speed(x = 0, y = 0, z = z_out, timeout=1)
                else:
                    ep_chassis1.drive_speed(x = 0, y = 0, z = 2.25*z_out, timeout=1)
                    time.sleep(1)
                    z_out = 0
                    ep_chassis1.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                    counter1 = 2
                    time.sleep(0.5)
            else:
                ep_chassis1.drive_speed(x = 0, y = 0, z = 5, timeout=1)
        elif counter1 == 2: # approach river 
            print("approaching river")
            if abs(error_fb) > 5:
                x_out = Kx*error_fb
                ep_chassis1.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
            else:
                ep_chassis1.drive_speed(x = 2*x_out, y = 0, z = 0, timeout=1)
                time.sleep(3)
                x_out = 0
                ep_chassis1.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                counter1 = 3
                counter2 = 0
        if counter1 == 4:
            ep_gripper1.open(power=100)
            time.sleep(1)
            ep_gripper2.pause()
            counter2 = 6
        if counter1==4:
            ep_camera1 = ep_robot2.camera
            ep_camera1.stop_video_stream()
        ####################### ROBOT 2########################################################
        if cameraFLag != 0:
            frame = ep_camera2.read_cv2_image(strategy="newest", timeout=2.5)
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

        if counter2 == 0: # rotating towards river
            if abs(theta) > 0:
                z_out = Kt*theta
                ep_chassis2.drive_speed(x = 0, y = 0, z = z_out, timeout=1)
            else:
                ep_chassis2.drive_speed(x = 0, y = 0, z = 2.25*z_out, timeout=1)
                time.sleep(1)
                z_out = 0
                ep_chassis2.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                counter2 = 1
                time.sleep(0.5)
        elif counter2 == 1: # wait for communication
            # message = str(socket.recv())
            # if message == "arrived":
            # time.sleep(20)
            counter2 = 2
        elif counter2 == 2: # center on other robot 
            FLAG =True
            while FLAG:
                frame = ep_camera2.read_cv2_image(strategy="newest", timeout=2.5)
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
                            if n2 == 0:
                                if (int(lego_center_x) - frame_center[0]) > 0:
                                    ep_chassis2.drive_speed(x = 0, y = .05, z = 0, timeout=10)
                                if (int(lego_center_x) - frame_center[0]) < 0:
                                    ep_chassis2.drive_speed(x = 0, y = -.05, z = 0, timeout=10)
                                if abs(lego_center_x - frame_center[0]) < 15:
                                    n2 = 1
                                    # ep_chassis.drive_speed(x = 0.05, y = 0, z = 0, timeout=10)
                                    # if lego_center_x > 300 and lego_center_x<342.0 and lego_center_y>200:      
                                    #     ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=5)
                                    counter2 = 3
                                    print("Approach River")
                                    FLAG = False
            cv2.destroyWindow("image0.jpg")
        
        # Approaching river
        elif counter2 == 3:
            if abs(error_fb_riv) > 5:
                x_out = Kx_riv*error_fb_riv
                print(x_out)
                ep_chassis2.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
            else:
                ep_chassis2.drive_speed(x = .1, y = 0, z = 0, timeout=10)
                time.sleep(2.5)
                x_out = 0
                ep_chassis2.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                counter2 = 4

        elif counter2 == 4: # send communication handoff is ready
            ep_gripper2.close(power=100)
            time.sleep(1)
            ep_gripper2.pause()  
        #   message = "True"
        #   socket.send(message)
        #   counter = 5
        # elif counter == 5: 
        #   message = str(socket.recv())
        #   if message == "open":
            # time.sleep(10)
            counter1 =4

        elif counter2 == 6:
            ep_chassis2.drive_speed(x = -.15, y = 0, z = 0, timeout=5)
            time.sleep(2)
            counter2 = 7
        elif counter2 == 7:
            ep_chassis2.drive_speed(x = 0, y = 0, z = 7.5, timeout=5)
            if abs(centero[0] - frame_center[0]) < 5:
                counter2 = 8
                ep_chassis2.drive_speed(x = 0, y = 0, z = 0, timeout=1)
        elif counter2 == 8:
            if abs(error_fb_end) > 5:
                x_out = Kx_end*error_fb_end
                ep_chassis2.drive_speed(x = x_out, y = 0, z = 0, timeout=1)
            else:
                ep_chassis2.drive_speed(x = 0, y = 0, z = 0, timeout=1)
                time.sleep(1)
                counter2 = 9
        elif counter2 == 9:
            ep_chassis2.drive_speed(x = .15, y = 0, z = 0, timeout=10)
            time.sleep(2.75)
            ep_chassis2.drive_speed(x = 0, y = 0, z = 0, timeout=1)
            counter2 = 10
        elif counter2 == 10:
            ep_arm2.moveto(x=180, y=-30).wait_for_completed()
            ep_gripper2.open(power=50)
            time.sleep(1)
            ep_gripper2.pause()
            counter2 = 11
        elif counter2 == 11:
            ep_chassis2.drive_speed(x = -.15, y = 0, z = 0, timeout=5)
            time.sleep(2)
            counter2 = 12
            ep_chassis2.drive_speed(x = 0, y = 0, z = 0, timeout=5)
        elif counter2 == 12:
            ep_chassis2.drive_speed(x = 0, y = 0, z = 20, timeout=10)        
        # elif counter1 == 3:
            # Robot_destiniation(3)
            Gripped = True
            # while not Gripped:
                # message = str(socket.recv())
                # if message == "True":
                    # Gripped = True
            time.sleep(3)
            ep_gripper1.sub_status(freq = 1, callback = send_gripper_status)
            ep_gripper1.open()
            time.sleep(3)
            ep_gripper1.pause()
            ep_chassis1.drive_speed(x = -.25, y = 0, z = 0, timeout=1)
            time.sleep(2)
            ep_chassis1.drive_speed(x = 0, y = 0, z = 0, timeout=1)
            
        
        cv2.rectangle(frame, (x_big, y_big), (x_big + w_big, y_big + h_big), (0,255,255), 4)
        cv2.circle(frame, center, 5, (0, 255, 255), -1)
        cv2.imshow("bounding",frame)
        cv2.waitKey(10)