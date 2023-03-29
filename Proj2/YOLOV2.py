from multiprocessing import Process, Queue
from ultralytics import YOLO
import cv2
import time
from robomaster import robot
from robomaster import camera


def get_frame(q):
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False, resolution=camera.STREAM_360P)

    while True:
        frame = ep_camera.read_cv2_image(strategy="newest", timeout=2)
        if frame is not None:
            q.put(frame)


def predict(q):
    model = YOLO("Project2-5\\runs\detect\\train12\weights\\best.pt")

    while True:
        if not q.empty():
            frame = q.get()
            start = time.time()
            if model.predictor:
                model.predictor.args.verbose = False
            results = model.predict(source=frame, show=True)
            end = time.time()
            print(1.0 / (end-start))


if __name__ == '__main__':
    q = Queue()
    p1 = Process(target=get_frame, args=(q,))
    p2 = Process(target=predict, args=(q,))

    p1.start()
    p2.start()

    p1.join()
    p2.join()
