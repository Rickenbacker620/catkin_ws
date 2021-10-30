#!/usr/bin/env python3

from types import SimpleNamespace
from core.direction_detector import DirectionDetector

import cv2 as cv
import rospy
from std_msgs.msg import Int32
from utils.camera import load_coefficients
from utils.camera import undistort_image
from utils.debug import image_sender


parameters = SimpleNamespace(weights='/home/yzu/catkin_ws/src/qingzhou_vision/src/YOLOv5/weights/last.pt',
                             source='YOLOv5/tsttt.png',
                             img_size=640,
                             conf_thres=0.25,
                             iou_thres=0.45,
                             device='cuda',
                             classes=None,
                             agnostic_nms=False,
                             augment=False,
                             update=False,
                             project='runs/detect',
                             name='exp',
                             exist_ok=False,
                             line_thickness=3,
                             hide_labels=False,
                             hide_conf=False)


if __name__ == "__main__":
    rospy.init_node('direction_detector', anonymous=True)
    detector = DirectionDetector(parameters)

    cap = cv.VideoCapture("/home/yzu/catkin_ws/SampleVid.mp4")

    cap.set(cv.CAP_PROP_POS_FRAMES, 1200)

    mtx, dist = load_coefficients(
        "/home/yzu/catkin_ws/src/qingzhou_vision/config/calibration_chessboard.yml")

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # if (cv.waitKey(27) == 'q'):
        #     break
        _, image = cap.read()

        if image.shape != (720, 1280):
            image = cv.resize(image, (1280, 720))
        undistort = cv.resize(undistort_image(image, mtx, dist), (640, 360))

        direction = detector(undistort, debug=True)
        print(direction)

        image_sender.send(detector.image_marked)

        rate.sleep()
