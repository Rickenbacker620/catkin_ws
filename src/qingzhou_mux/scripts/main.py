#!/usr/bin/env python3
from datetime import date
from logging import shutdown
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool


class SignalHandler:
    def __init__(self) -> None:
        rospy.init_node('mux_controller', anonymous=True)
        # self.nav_sub = rospy.Subscriber("nav_vel",Twist, self.nav_call_back, queue_size=1)
        self.bias_sub = rospy.Subscriber("lane_bias", Int32, self.bias_callback, queue_size=1)
        self.zebra_sub = rospy.Subscriber("has_zebra", Bool, self.zebra_callback, queue_size=1)

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.has_zebra = False
        self.bias = False

        self.rate = rospy.Rate(10)

    def nav_callback(self, twist):
        print(twist)

    def zebra_callback(self, has_zebra):
        self.has_zebra = has_zebra.data

    def bias_callback(self, bias):
        self.bias = bias.data

    def core_process(self):
        while not rospy.is_shutdown():
            order = Twist()
            turn = -0.003*(self.bias)
            velocity = 0.05

            if self.has_zebra is True:
                velocity = 0
            else:
                velocity = 0.05

            bias_dir = "Left" if self.bias > 0 else "Right"
            vel_dir = "Left" if turn > 0 else "Right"

            # print(f"Bias {bias_dir} {abs(self.bias)}, Turning {vel_dir} {abs(turn)}")

            order.angular.z = turn

            print(self.has_zebra, velocity)

            order.linear.x = velocity

            self.vel_pub.publish(order)

            self.rate.sleep()


if __name__ == "__main__":
    mux = SignalHandler()
    mux.core_process()
