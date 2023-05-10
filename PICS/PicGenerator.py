import time
import cv2
import numpy as np
from robomaster import robot
from robomaster import camera

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_chassis = ep_robot.chassis
    ep_arm = ep_robot.robotic_arm
    ep_arm.moveto(x=120, y=45).wait_for_completed()
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
    
    for i in range(385,500):
        if i%4==0:
            ep_chassis.drive_speed(x = .3, y = 0, z = 0, timeout=1)
        elif i%4==1:
            ep_chassis.drive_speed(x = -.3, y = 0, z = 0, timeout=1)
        elif i%4==2:
            ep_chassis.drive_speed(x = 0, y = 0, z = 10, timeout=1)
        elif i%4==3:
            ep_chassis.drive_speed(x = 0, y = 0, z = -10, timeout=1)
            # print('count')
        
        # input("Press Enter . . .")
        time.sleep(1)
        print(i)
        print("taking photo")
        ep_chassis.drive_speed(x = 0, y = 0, z = 0, timeout=1)
        img = ep_camera.read_cv2_image(strategy="newest", timeout=1)
        time.sleep(2)
        filename = str(i)
        filename = filename + ".jpg"
        cv2.imwrite(filename, img)
        print("done")
    ep_camera.stop_video_stream()
    ep_robot.close()
    print ('Exiting')
    exit(1)