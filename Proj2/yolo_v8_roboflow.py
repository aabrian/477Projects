from roboflow import Roboflow
import cv2
import numpy as np
import time

from robomaster import robot
from robomaster import camera
ROBOFLOW_API_KEY = "kKusTXhj0ObVGmi9slHp"
ROBOFLOW_MODEL = "project2-l7rdy/4" # eg xx-xxxx--#
ROBOFLOW_SIZE = 360

rf = Roboflow(api_key="kKusTXhj0ObVGmi9slHp")
project = rf.workspace().project("project2-l7rdy")
model = project.version(4).model

def call_roboflow_api(img):
    cv2.imwrite('test.jpg', img)
    resp = model.predict('test.jpg', confidence=40, overlap=30).json()
    return resp

# Use vid instead of ep_camera to use your laptop's webcam
# vid = cv2.VideoCapture(0)

ep_robot = robot.Robot()
ep_robot.initialize(conn_type="ap")
ep_camera = ep_robot.camera
ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

while True:
    # Get the current image from the webcam
    # ret, image = vid.read()
    image = ep_camera.read_cv2_image(strategy="newest", timeout=5.0)

    start = time.time()
    response = call_roboflow_api(image)
    print(response)
    print(1.0/(time.time() - start))

    if image is not None:
      cv2.imshow('image', image)
      cv2.waitKey(1)
