from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor

model = YOLO('yolov8s.pt')

results = model.predict(source = "0",show=True)

print(results)