#!/usr/bin/env python3
import cv2 as cv
import rospy
from std_msgs.msg import Int32
from utils.camera import gstreamer_pipeline, load_coefficients
from core.lane_detector import LaneDetector
from core.parameters import Parameters
from utils.camera import undistort_image
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from utils.debug import ImageSender



def process(image):
    image = bridge.imgmsg_to_cv2(image, "bgr8")

    bias = detector(image, debug=True)

    if bias is None:
        bias = 0

    sender.send(detector.image_marked)

    pub.publish(bias)


if __name__ == "__main__":
    bridge = CvBridge()

    sender = ImageSender('192.168.2.143', 8989)

    rospy.init_node('lane_detector', anonymous=True)

    sub = rospy.Subscriber('camera_image', Image, process, queue_size=1)
    pub = rospy.Publisher('lane_bias', Int32, queue_size=1)

    parameters = Parameters()
    detector = LaneDetector(parameters)

    rospy.spin()

    # rate = rospy.Rate(10)
