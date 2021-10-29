#!/usr/bin/env python3
import os
import cv2 as cv
import rospy
from detectors.detector import ImageDetector
from geometry_msgs.msg import Twist
import socket

WIDTH = 1920
HEIGHT = 1080

VIDEO = True


def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=10,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


if __name__ == "__main__" and VIDEO is True:
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('lane_detector', anonymous=True)

    # cap = cv.VideoCapture("./output.avi")

    cap = cv.VideoCapture(gstreamer_pipeline(flip_method=0), cv.CAP_GSTREAMER)

    #cap = cv.VideoCapture("./SampleVid.mp4")
    detect = ImageDetector()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # if (cv.waitKey(27) == 'q'):
        #     break
        _, image = cap.read()

        detect.set_image(image)
        bias = detect.lane_detetcor(detect.image_raw, debug=True)
        if bias is None:
            bias = 0
        order = Twist()
        order.angular.z = -0.003*bias
        print(bias)
        order.linear.x = 0.05

        pub.publish(order)

        rate.sleep()

        # detect.preprocess()

if __name__ == "__main__" and VIDEO is False:
    for file in os.listdir("./tar"):
        image = cv.imread("./tar/" + file)
        detect = ImageDetector(image)
        detect.preprocess()
        cv.waitKey(0)
