from ultralytics import YOLO
import cv2
import time
from robomaster import robot
from robomaster import camera
def pickup():
    ep_gripper.open(power=100)
    # ep_arm.moveto(x=100, y=40).wait_for_completed()
    ep_arm.moveto(x=170, y=40).wait_for_completed()

    ##close gripper
    ep_gripper.close(power=100)
    time.sleep(1)
    ep_gripper.pause()
    ep_arm.moveto(x=170, y=-0).wait_for_completed()


if __name__ == '__main__':
    model = YOLO("Project2-5\\runs\detect\\train12\weights\\best.pt")
    # Use vid instead of ep_camera to use your laptop's webcam
    # vid = cv2.VideoCapture(0)
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_arm = ep_robot.robotic_arm
    ep_gripper = ep_robot.gripper
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)
    #get it into the walking position
    while True:
        # ret, frame = vid.read()
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=2)
        if frame is not None:
            start = time.time()
            if model.predictor:
                model.predictor.args.verbose = False
            results = model.predict(source=frame, show=True,half=True)
            boxes = results[0].boxes
            if len(boxes)>0:
                box = boxes[0].xyxy  # returns one box
                # print(box)
                lego_center_x = ((box[0,0]+box[0,2])/2).item()
                lego_center_y = ((box[0,1]+box[0,3])/2).item()

                ## close gripper when in range
                print('X=',lego_center_x)
                print('Y= ',lego_center_y)
                if lego_center_x >300.0 and lego_center_x<342.0 and lego_center_y>142:
                    print('grab')      
                    continue
                    break
                    
            end = time.time()
            print(1.0 / (end-start))
    ep_camera.stop_video_stream()
    print('gripping')
    # ep_gripper.close(power=100)
    pickup()     
    time.sleep(1)

    print('done!')
    # ep_robot.close()
    exit()