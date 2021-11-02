#!/usr/bin/env python3
from datetime import date
from logging import shutdown
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32, Bool, String
from sensor_msgs.msg import LaserScan
from enum import Enum, auto


class SignalHandler:
    TURN_LEFT = 0
    TURN_RIGHT = 1
    GO_STRAIGHT = 2
    AVOID_OBSTACLE = 3

    def __init__(self) -> None:
        rospy.init_node('mux_controller', anonymous=True)
        self.bias_sub = rospy.Subscriber("lane_bias", Int32, self.bias_callback, queue_size=1)
        self.zebra_sub = rospy.Subscriber("has_zebra", Bool, self.zebra_callback, queue_size=1)
        self.dir_sub = rospy.Subscriber("scan", LaserScan, self.laser_scan_callback, queue_size=1)
        self.obstacle_detect = rospy.Subscriber("direction", String, self.direction_callback, queue_size=1)
        self.light_sub = rospy.Subscriber("traffic_light", String, self.traffic_light_callback, queue_size=1)
        self.nav_sub = rospy.Subscriber("nav_vel", Twist, self.nav_callback, queue_size=1)

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.nav_vel = Twist()
        self.has_zebra = False
        self.bias = False
        self.rate = rospy.Rate(10)
        self.has_obstacle = False
        self.direction = "None"
        self.traffic_light = "None"

        self.state = 2

    def laser_scan_callback(self, laser: LaserScan):
        obs_angular_range = 20
        obs_liner_threshold = 0.8
        beam_count_threshold = 35
        has_obstacle_array = [beam < obs_liner_threshold for beam in
                              laser.ranges[360 - obs_angular_range:360+obs_angular_range]]
        self.has_obstacle = has_obstacle_array.count(True) > beam_count_threshold

    def get_status(self):
        if self.has_obstacle == True or self.has_zebra == True:
            if self.direction in ["Left", "Left-Straight"]:
                self.state = self.TURN_LEFT
            elif self.direction in ["Right", "Right-Straight"]:
                self.state = self.TURN_RIGHT
            else:
                self.state = self.AVOID_OBSTACLE
        elif self.direction == "Right":
            self.state = self.TURN_RIGHT
        elif self.direction == "Left":
            self.state = self.TURN_LEFT
        else:
            self.state = self.GO_STRAIGHT

    def do_turn_left(self):
        goal = PoseStamped()

    def nav_callback(self, twist):
        self.nav_vel = twist

    def direction_callback(self, direction):
        self.has_zebra = direction.data

    def traffic_light_callback(self, light):
        self.traffic_light = light.data

    def zebra_callback(self, has_zebra):
        self.has_zebra = has_zebra.data

    def bias_callback(self, bias):
        self.bias = bias.data

    def core_process(self):
        while not rospy.is_shutdown():
            order = Twist()
            turn = -0.003*(self.bias)
            velocity = 0.05

            # if self.has_zebra is True:
            #     velocity = 0
            # else:
            #     velocity = 0.05
            velocity = 0.1

            bias_dir = "Left" if self.bias > 0 else "Right"
            vel_dir = "Left" if turn > 0 else "Right"

            print(f"Bias {bias_dir} {abs(self.bias)}, Turning {vel_dir} {abs(turn)}")

            order.angular.z = turn

            # print(self.has_zebra, velocity)

            order.linear.x = velocity

            self.vel_pub.publish(order)
            # self.get_status()
            # print(self.state)

            self.rate.sleep()


if __name__ == "__main__":
    mux = SignalHandler()
    mux.core_process()
