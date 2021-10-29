#!/usr/bin/env python3

import cv2
import sys
import socket
# import torch
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String


# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen

# image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)


def gstreamer_pipeline(
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=60,
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


# class camera_node():


def show_camera():
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(
        flip_method=0), cv2.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("CSI Camera", 0) >= 0:
            ret_val, img = cap.read()
            # cv2.imshow("CSI Camera", img)
            # from cv_bridge import CvBridge
            # bridge = CvBridge()
            # cv_image = bridge.imgmsg_to_cv2(
            #     img, desired_encoding='passthrough')
            # image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))

            # This also acts as
            keyCode = cv2.waitKey(30) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")


# if __name__ == "__main__":
#     bridge = CvBridge()
#     rospy.init_node('image_sender', anonymous=True)
#     pub = rospy.Publisher('img_topic_test', Image, queue_size=10)
#     cap = cv2.VideoCapture(gstreamer_pipeline(
#         flip_method=0), cv2.CAP_GSTREAMER)
#     rate = rospy.Rate(10)  # 10hz
#     if cap.isOpened():
#         while not rospy.is_shutdown():
#             ret_val, image = cap.read()
#             if ret_val is not True:
#                 break
#             # pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
#             val = bridge.cv2_to_imgmsg(image, "bgr8")
#             pub.publish(val)
#             rate.sleep()
#     else:
#         print("end")

# if __name__ == "__main__":
#     bridge = CvBridge()
#     rospy.init_node('image_sender', anonymous=True)
#     pub = rospy.Publisher('img_topic_test', Image, queue_size=1)
#     cap = cv2.VideoCapture(gstreamer_pipeline(
#         flip_method=0), cv2.CAP_GSTREAMER)
#     rate = rospy.Rate(1)  # 10hz
#     if cap.isOpened():
#         while not rospy.is_shutdown():
#             ret_val, image = cap.read()
#             if ret_val is not True:
#                 break
#             pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
#             # image = cv2.imread(f'/home/yzu/catkin_ws/src/qingzhou_vision/scripts/images/test.png')
#             # pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
#             rate.sleep()
#         # while not rospy.is_shutdown():
#         #     ret_val, image = cap.read()
#         #     if ret_val is not True:
#         #         break
#         #     # pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
#         #     val = bridge.cv2_to_imgmsg(image, "bgr8")
#         #     pub.publish(val)
#         #     rate.sleep()
#     # else:
#     #     print("end")


if __name__ == "__main__":
    cap = cv2.VideoCapture(gstreamer_pipeline(
        flip_method=0), cv2.CAP_GSTREAMER)

    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # out = cv2.VideoWriter('output.avi', fourcc, 20.0, (1280, 720))

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # cap = cv2.VideoCapture(0,cv2.CAP_GSTREAMER)
    cap = cv2.VideoCapture(gstreamer_pipeline(
        flip_method=0), cv2.CAP_GSTREAMER)
    while True:
        ret, img = cap.read()
        if ret:
            img = cv2.resize(img, (341, 256))  # 测试这个尺寸图像比较稳定，再大发送不稳定
            img = cv2.imencode('.jpeg', img)[1]      # 使用jpeg格式压缩图像
            data_encode = np.array(img)
            data = data_encode.tostring()   # 转换为byte
            # cv2.waitKey(0)
            print(data)
            s.sendto(data, ('192.168.2.109', 8989))

    while(cap.isOpened()):
        ret_val, image = cap.read()
        if ret_val is not True:
            break
        out.write(image)

    cap.release()
    out.release()

    # while not rospy.is_shutdown():
    #     ret_val, image = cap.read()
    #     if ret_val is not True:
    #         break
    #     pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
    # image = cv2.imread(f'/home/yzu/catkin_ws/src/qingzhou_vision/scripts/images/test.png')
    # pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
    rate.sleep()


# if __name__ == "__main__":
#     talker()

    # while True:
    #     print("hello")

    # print(gstreamer_pipeline(flip_method=0))
    # cap = cv2.VideoCapture(gstreamer_pipeline(
    #     flip_method=0), cv2.CAP_GSTREAMER)
    # if cap.isOpened():
    # rospy.init_node('image_converter', anonymous=True)
    # show_camera()
    # try:
    #     rospy.spin()
    # except KeyboardInterrupt:
    #     print("shutting down")
    # cv2.destroyAllWindows()
