#!/usr/bin/env python3
import cv2 as cv
import rospy
from std_msgs.msg import Int32
from utils.camera import gstreamer_pipeline, load_coefficients
from core.lane_detector import LaneDetector
from core.parameters import Parameters
from utils.camera import undistort_image

WIDTH = 1920
HEIGHT = 1080

VIDEO = True




if __name__ == "__main__" and VIDEO is True:
    rospy.init_node('lane_detector', anonymous=True)
    pub = rospy.Publisher('lane_bias', Int32, queue_size=1)
    parameters = Parameters()
    detector = LaneDetector(parameters)

    # cap = cv.VideoCapture("./output.avi")
    # cap = cv.VideoCapture(gstreamer_pipeline(), cv.CAP_GSTREAMER)
    cap = cv.VideoCapture("./SampleVid.mp4")
    rate = rospy.Rate(10)

    mtx, dist = load_coefficients("/home/shiro/ROSProjs/catkin_ws/src/qingzhou_vision/config/calibration_chessboard.yml")

    while not rospy.is_shutdown():
        # if (cv.waitKey(27) == 'q'):
        #     break
        _, image = cap.read()


        if image.shape != (720, 1280):
            image = cv.resize(image, (1280, 720))
        undistort = cv.resize(undistort_image(image, mtx, dist), (640, 360))

        bias = detector(undistort,debug=True)

        if bias is None:
            bias = 0

        pub.publish(bias)

        rate.sleep()