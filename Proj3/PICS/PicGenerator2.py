import time
import cv2
import numpy as np
from robomaster import robot
from robomaster import camera
import keyboard
global i
i = 328
def take_pic(i):
        print(i)
        # print("taking photo")
        time.sleep(.1)
        img = ep_camera.read_cv2_image(strategy="newest", timeout=4)
        # time.sleep(2)
        filename = str(i)
        filename = "\\Proj3\\PICS\\"+filename + ".jpg"
        cv2.imwrite(filename, img)
        # print("done")
        
if __name__ == '__main__':
    while True:
        ep_robot = robot.Robot()
        ep_robot.initialize(conn_type="ap")
        ep_chassis = ep_robot.chassis
        ep_camera = ep_robot.camera
        ep_camera.start_video_stream(display=False, resolution=camera.STREAM_720P)
        if keyboard.is_pressed('up'):
            # print('up')
            ep_chassis.drive_speed(x = .30,y = 0,z = 0,timeout = .3)
            take_pic(i)
            i = i+1
            time.sleep(.5)
        if keyboard.is_pressed('down'):
            ep_chassis.drive_speed(x = -.30,y = 0,z = 0,timeout = .3)  
            i = take_pic(i) 
            i = i+1 
            # print('down')
            time.sleep(.5)
        if keyboard.is_pressed('left'):
            ep_chassis.drive_speed(x = 0,y = -.30,z = 0,timeout = .3)
            take_pic(i)
            i = i+1
            # print('left')
            time.sleep(.5)
        if keyboard.is_pressed('right'):
            ep_chassis.drive_speed(x = 0,y = .3,z = 0,timeout = .3)
            take_pic(i)
            i = i+1
            # print('right')
            time.sleep(.5)
        if keyboard.is_pressed('a'):
            ep_chassis.drive_speed(x = 0,y = 0,z = -15,timeout = .3)   
            take_pic(i)
            i = i+1
            time.sleep(.5)
        if keyboard.is_pressed('d'):
            ep_chassis.drive_speed(x = 0,y = 0,z =15,timeout = .3) 
            take_pic(i)
            i = i+1
            time.sleep(.5)
        else:
            ep_chassis.drive_speed(x = 0,y = 0,z = 0,timeout = .3)
            # take_pic(i)
            # print('still')
            time.sleep(.1)

    ep_camera.stop_video_stream()
    ep_robot.close()
    print ('Exiting')
    exit(1)