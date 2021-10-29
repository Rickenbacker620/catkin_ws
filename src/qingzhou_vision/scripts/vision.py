#!/usr/bin/env python3
# license removed for brevity

import rospy
# import torch
from asd.util import say_hello
# from std_msgs.msg import String


# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10)  # 10hz
#     while not rospy.is_shutdown():
#         # hello_str = "hello world %s" % rospy.get_time()
#         hello_str = torch.__version__
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# import sys


if __name__ == '__main__':
    # print(sys.version)
    # print("hello")
    say_hello()

    # try:
    #     talker()
    # except rospy.ROSInterruptException:
    #     pass
