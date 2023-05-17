from ultralytics import YOLO
from robomaster import robot
from robomaster import camera
import numpy as np
import time
import cv2

def sub_position_handler(p, x_new):
    x_new[0] = p[0]
    x_new[1] = p[1]
    x_new[2] = p[2]
    # print("chassis position: x: {}".format(x_new))

def pickup():
    ep_gripper.open(power=50)
    time.sleep(1)
    ep_gripper.pause()
    

    ep_gripper.close(power=100)
    time.sleep(1)
    ep_gripper.pause()
    
    ep_arm.moveto(x=170, y=0).wait_for_completed()

if __name__ == '__main__':
    x_new = np.zeros((3,))

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    ep_chassis = ep_robot.chassis
    ep_robot.chassis.sub_position(freq=25, callback=lambda p: sub_position_handler(p, x_new))
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_sensor = ep_robot.sensor

    model = YOLO("477\\Project2-5\\runs\detect\\train12\weights\\best.pt")

    counter = 0
    y_speed = 0.15
    n = 0

    while True:

        if counter == 0:
            ep_arm.moveto(x=180, y=-70).wait_for_completed()
            ep_gripper.open(power=50)
            time.sleep(1)
            ep_gripper.pause()
            ep_chassis.drive_speed(x = 0, y = -0.25, z = 0, timeout=10)
            time.sleep(5.5)
            ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=10)
            x_old = x_new
            counter += 1

        if counter == 1:
            FLAG =True
            while FLAG:
                frame = ep_camera.read_cv2_image(strategy="newest", timeout=2.5)
                frame_center = (int(frame.shape[1]/2),int(frame.shape[0]/2))
                if frame is not None:
                    if model.predictor:
                        model.predictor.args.verbose = False
                    results = model.predict(source=frame, show=True,half=True)
                    boxes = results[0].boxes
                    if n == 0:
                        ep_chassis.drive_speed(x = 0, y = -.05, z = 0, timeout=10)
                    for box in results[0].boxes:
                        print(results[0].names[int(box.cls.cpu().numpy())])
                        if 'lego' in results[0].names[int(box.cls.cpu().numpy())]:
                            box = box.xyxy
                            lego_center_x = ((box[0,0]+box[0,2])/2).item()
                            lego_center_y = ((box[0,1]+box[0,3])/2).item()
                            print(lego_center_x-frame_center[0])
                            start_t = time.time()
                            ep_chassis.drive_speed(x = 0, y = -.05, z = 0, timeout=10)
                            
                            if abs(int(lego_center_x) - frame_center[0]) < 25:
                                ep_chassis.drive_speed(x=.05, y = 0, z = 0, timeout=5)
                                time.sleep(2.5)
                                ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=5)
                                counter += 1
                                FLAG = False
                                pickup()
                                cv2.destroyWindow("image0.jpg")
        if counter == 2:
            ep_chassis.drive_speed(x=-.05, y = 0, z = 0, timeout=5)
            time.sleep(2)
            ep_chassis.drive_speed(x = 0, y = 0.25, z = 0, timeout=10)
            time.sleep(6)
            ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=10)
            counter += 1

        if counter == 3:
            
            time.sleep(10)