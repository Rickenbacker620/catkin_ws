#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32


class SignalHandler:
    def __init__(self) -> None:
        rospy.init_node('mux_controller', anonymous=True)
        # self.nav_sub = rospy.Subscriber("nav_vel",Twist, self.nav_call_back, queue_size=1)
        self.bias_sub = rospy.Subscriber("lane_bias", Int32, self.bias_callback, queue_size=1)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def nav_call_back(self, twist):
        print(twist)

    def bias_callback(self, bias):
        order = Twist()
        order.angular.z = -0.001*(bias.data)
        order.linear.x = 0.05
        self.vel_pub.publish(order)


if __name__ == "__main__":
    mux = SignalHandler()
    rospy.spin()
