#!/usr/bin/env python3

from types import SimpleNamespace
from core.direction_detector import DirectionDetector

import cv2 as cv
import rospy
from std_msgs.msg import Int32
from utils.camera import load_coefficients
from utils.camera import undistort_image
from utils.debug import image_sender
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge, CvBridgeError
from utils.debug import ImageSender

bridge = CvBridge()


class DirectionNode:
    def __init__(self) -> None:
        self.bridge = CvBridge()

        self.sender = ImageSender('192.168.2.143', 8989)

        rospy.init_node('direction_detect', anonymous=True)

        self.sub = rospy.Subscriber('camera_image', Image, self.process, queue_size=1)
        self.pub = rospy.Publisher('direction', String, queue_size=1)

        self.detector = DirectionDetector(parameters)

    def process(self, image):
        image = bridge.imgmsg_to_cv2(image, "bgr8")

        direction = self.detector(image, debug=True)

        self.sender.send(self.detector.image_marked)

        self.pub.publish(direction)

        # pub.publish(bias)


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
    node = DirectionNode()
    rospy.spin()
