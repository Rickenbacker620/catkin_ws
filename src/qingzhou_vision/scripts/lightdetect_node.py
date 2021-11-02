#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from core.parameters import Parameters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def detector(image, debug):
    return "Green"


def process(image):
    image = bridge.imgmsg_to_cv2(image, "bgr8")

    light = detector(image, debug=True)

    light_pub.publish(light)


if __name__ == "__main__":
    bridge = CvBridge()

    rospy.init_node('light_detect', anonymous=True)

    sub = rospy.Subscriber('camera_image', Image, process, queue_size=1)
    light_pub = rospy.Publisher('traffic_light', String, queue_size=1)

    rospy.spin()

    # rate = rospy.Rate(10)
