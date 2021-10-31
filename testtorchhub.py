import torch
import cv2 as cv

# Model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='./last.pt')  # or yolov5m, yolov5l, yolov5x, custom


# Images
# img = 'https://ultralytics.com/images/zidane.jpg'  # or file, Path, PIL, OpenCV, numpy, list
img = cv.imread("./test.png")

# Inference
results = model(img)

# # Results
# results.pandas().xyxy[0]  # or .show(), .save(), .crop(), .pandas(), etc.
