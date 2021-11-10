import numpy as np
import cv2
from utils.camera import gstreamer_pipeline
from utils.debug import ImageSender

sender = ImageSender("192.168.2.109", 8989)

cap = cv2.VideoCapture(gstreamer_pipeline())

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 10.0, (1280, 720))

while(cap.isOpened()):
    try:
        ret, frame = cap.read()
        if ret == True:
            out.write(frame)
            sender.send(frame)
        else:
            break
    except KeyboardInterrupt:
        break
cap.release()
out.release()
