#!/usr/bin/env python3
from utils.camera import gstreamer_pipeline, load_coefficients, undistort_image
import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':

    rospy.init_node('image_pub', anonymous=True)

    image_pub = rospy.Publisher("camera_image", Image, queue_size=1)

    # cap = cv.VideoCapture("/home/yzu/catkin_ws/SampleVid.mp4")
    # cap.set(cv.CAP_PROP_POS_FRAMES, 2500)
    cap = cv.VideoCapture(gstreamer_pipeline())

    mtx, dist = load_coefficients(
        "/home/yzu/catkin_ws/src/qingzhou_vision/config/calibration_chessboard.yml")

    bridge = CvBridge()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        _, image = cap.read()
        if image.shape != (720, 1280):
            image = cv.resize(image, (1280, 720))

        # 去除畸变
        undistort = cv.resize(undistort_image(image, mtx, dist), (640, 360))

        cv.putText(undistort, str(cap.get(cv.CAP_PROP_POS_FRAMES)), (200, 200),
                   cv.FONT_HERSHEY_COMPLEX, 2.0, (255, 255, 255))

        try:
            image_pub.publish(bridge.cv2_to_imgmsg(undistort, "bgr8"))
        except CvBridgeError as e:
            print(e)
        rate.sleep()
