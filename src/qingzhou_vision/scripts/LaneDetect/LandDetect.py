import cv2 as cv
import numpy as np
from .common import *


WIDTH = 1920
HEIGHT = 1080


def detect_line(image):
    image = cv.resize(image, (WIDTH, HEIGHT))

    roi = image[int(HEIGHT*0.7):, ...]

    # 寻找左右车道线
    ((k1, b1), (k2, b2)) = find_lane(roi)

    b1 += HEIGHT*0.7
    b2 += HEIGHT*0.7

    return (k1, b1), (k2, b2)


if __name__ == '__main__':
    cap = cv.VideoCapture('./LaneDetect/SampleVid.mp4')

    fourcc = cv.VideoWriter_fourcc(*'XVID')
    out = cv.VideoWriter('output.avi', fourcc, 20.0, (1920, 1080))

    while cap.isOpened():

        ret, frame = cap.read()
        if ret is False:
            print("Video Ended or Corrupted")
            break

        (k1, b1), (k2, b2) = detect_line(frame)

        pt1, pt2 = kb2pt(frame, k1, b1)
        pt3, pt4 = kb2pt(frame, k2, b2)

        points = get_test_points(k1, b1, k2, b2)

        cv.imshow('window', frame)

        cv.waitKey(21)

        # out.write(frame)
