#!/usr/bin/env python3
import time
from logging import shutdown
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32, Bool, String
from sensor_msgs.msg import LaserScan
from simple_pid import PID
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib


class SignalHandler:
    TURN_LEFT = "L"
    TURN_RIGHT = "R"
    GO_STRAIGHT = "S"
    AVOID_OBSTACLE = "A"
    STOP = "P"

    def __init__(self) -> None:
        rospy.init_node('mux_controller', anonymous=True)
        self.bias_sub = rospy.Subscriber("lane_bias", Int32, self.bias_callback, queue_size=1)
        self.zebra_sub = rospy.Subscriber("has_zebra", Bool, self.zebra_callback, queue_size=1)
        self.dir_sub = rospy.Subscriber("scan", LaserScan, self.laser_scan_callback, queue_size=1)
        self.obstacle_detect = rospy.Subscriber("direction", String, self.direction_callback, queue_size=1)
        self.light_sub = rospy.Subscriber("traffic_light", String, self.traffic_light_callback, queue_size=1)
        self.nav_sub = rospy.Subscriber("nav_vel", Twist, self.nav_callback, queue_size=1)

        self.key_sub = rospy.Subscriber("key_vel", Twist, self.key_callback, queue_size=1)

        self.state_sub = rospy.Subscriber("state", String, self.state_callback, queue_size=1)

        self.state_pub = rospy.Publisher("state", String, queue_size=1)

        self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        self.nav_vel = Twist()
        self.key_vel = Twist()
        self.has_zebra = False
        self.bias = False
        self.rate = rospy.Rate(10)
        self.has_obstacle = False
        self.direction = "None"
        self.traffic_light = "None"

        self.state = self.GO_STRAIGHT

        self.last_time = time.time()

        self.pid = PID(0.001, 0, 0.0001, setpoint=0, output_limits=(-0.03, 0.03))

    def laser_scan_callback(self, laser: LaserScan):
        obs_angular_range = 20
        obs_liner_threshold = 1.2
        beam_count_threshold = 35
        has_obstacle_array = [beam < obs_liner_threshold for beam in
                              laser.ranges[360 - obs_angular_range:360+obs_angular_range]]
        self.has_obstacle = has_obstacle_array.count(True) > beam_count_threshold

    def state_callback(self, state: String):

        if state.data in ["L", "R", "S", "A", "P"]:
            self.state = state.data

    def get_status(self):
        if self.has_obstacle == True:
            if self.direction in ["Left", "Left-Straight"]:
                self.state = self.TURN_LEFT
            elif self.direction in ["Right", "Right-Straight"]:
                self.state = self.TURN_RIGHT
            else:
                self.state = self.AVOID_OBSTACLE
        elif self.traffic_light == "Red":
            self.state = self.STOP
        else:
            self.state = self.GO_STRAIGHT

    # 接受节点信息
    def key_callback(self, twist):
        self.key_vel = twist

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
    # 接受节点信息

    def handle_left(self):
        self.nav_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'

        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.position.y = 1.0
        goal.target_pose.pose.orientation.w = 1.0

        self.nav_client.send_goal(goal)
        self.nav_client.wait_for_result(timeout=rospy.Duration(3))

        if self.nav_client.get_state() != actionlib.SimpleGoalState.DONE:
            print("turn left not done")

        self.vel_pub.publish(self.nav_vel)

    def handle_right(self):
        self.nav_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'

        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.position.y = 1.0
        goal.target_pose.pose.orientation.w = 1.0

        self.nav_client.send_goal(goal)
        self.nav_client.wait_for_result(timeout=rospy.Duration(3))

        if self.nav_client.get_state() != actionlib.SimpleGoalState.DONE:
            print("turn right not done")

        self.vel_pub.publish(self.nav_vel)

    def handle_straight(self):
        dt = time.time() - self.last_time
        self.last_time = time.time()

        turn = self.pid(self.bias, dt)
        velocity = 0.2

        bias_dir = "Left" if self.bias > 0 else "Right"
        vel_dir = "Left" if turn > 0 else "Right"

        order = Twist()
        order.angular.z = turn
        order.linear.x = velocity

        print(f"Bias {bias_dir} {self.bias}, Turning {vel_dir} {turn}")

        # print(self.has_zebra, velocity)

        self.vel_pub.publish(order)

    def handle_stop(self):
        order = Twist()

        self.vel_pub.publish(order)

    def handle_obstacle(self):
        self.nav_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'

        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.orientation.w = 1.0

        self.nav_client.send_goal(goal)
        self.nav_client.wait_for_result(timeout=rospy.Duration(3))

        print(self.nav_client.get_state())
        if self.nav_client.get_state() != actionlib.SimpleGoalState.DONE:
            print("not done")

        self.vel_pub.publish(self.nav_vel)

    def core_process(self):
        debug = False
        # debug = True
        while not rospy.is_shutdown():
            self.get_status()
            print(self.state)
            if self.key_vel.linear.x < 0:
                self.vel_pub.publish(Twist())
            elif self.key_vel == Twist():
                if self.state == self.GO_STRAIGHT:
                    self.handle_straight()
                elif self.state == self.TURN_LEFT and debug == True:
                    self.handle_left()
                elif self.state == self.TURN_RIGHT and debug == True:
                    self.handle_right()
                elif self.state == self.STOP:
                    self.handle_stop()
                elif self.state == self.AVOID_OBSTACLE and debug == True:
                    self.handle_obstacle()
                else:
                    continue
            else:
                self.vel_pub.publish(self.key_vel)

            self.rate.sleep()


if __name__ == "__main__":
    mux = SignalHandler()
    mux.core_process()
