from roboflow import Roboflow
import cv2
rf = Roboflow(api_key="kKusTXhj0ObVGmi9slHp")
project = rf.workspace().project("project2-l7rdy")
model = project.version(4).model

# infer on a local image
print(model.predict("your_image.jpg", confidence=40, overlap=30).json())

# visualize your prediction
model.predict("your_image.jpg", confidence=40, overlap=30).save("prediction.jpg")

# infer on an image hosted elsewhere
# print(model.predict("URL_OF_YOUR_IMAGE", hosted=True, confidence=40, overlap=30).json())