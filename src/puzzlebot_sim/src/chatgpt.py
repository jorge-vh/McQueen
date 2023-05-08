#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class Bug0:
    def __init__(self):
        rospy.init_node('bug0')
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.rate = rospy.Rate(10) # Hz
        self.dist_tol = 0.1 # m
        self.angle_tol = math.pi/30 # radians
        self.goal = None
        self.obstacle = None
        self.odom = None
        self.cmd_vel = Twist()
        rospy.spin()

    def odom_callback(self, msg):
        self.odom = msg.pose.pose

    def scan_callback(self, msg):
        min_range = msg.range_max
        min_angle = None
        for i, r in enumerate(msg.ranges):
            if r < min_range:
                min_range = r
                min_angle = msg.angle_min + i * msg.angle_increment
        self.obstacle = (min_range, min_angle)

    def set_goal(self, x, y):
        self.goal = (x, y)
        self.cmd_vel.linear.x = 0.5
        self.cmd_vel.angular.z = 0
        self.pub.publish(self.cmd_vel)
        self.rate.sleep()


    def bug0(self):
        while not rospy.is_shutdown():
            if self.goal is None or self.odom is None or self.obstacle is None:
                self.rate.sleep()
                continue
            dist = math.sqrt((self.goal[0]-self.odom.position.x)**2 + (self.goal[1]-self.odom.position.y)**2)
            if dist < self.dist_tol:
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0
                self.pub.publish(self.cmd_vel)
                break
            if self.obstacle[0] > dist:
                angle_to_goal = math.atan2(self.goal[1]-self.odom.position.y, self.goal[0]-self.odom.position.x)
                angle_diff = angle_to_goal - math.atan2(self.odom.orientation.y, self.odom.orientation.x)
                if angle_diff > math.pi:
                    angle_diff -= 2*math.pi
                elif angle_diff < -math.pi:
                    angle_diff += 2*math.pi
                if abs(angle_diff) > self.angle_tol:
                    self.cmd_vel.linear.x = 0
                    self.cmd_vel.angular.z = angle_diff
                    self.pub.publish(self.cmd_vel)
                else:
                    self.cmd_vel.linear.x = 0.5
                    self.cmd_vel.angular.z = 0
                    self.pub.publish(self.cmd_vel)
            else:
                angle_to_obstacle = self.obstacle[1]
                if abs(angle_to_obstacle) < math.pi/2:
                    self.cmd_vel.linear.x = 0
                    self.cmd_vel.angular.z = -0.5
                else:
                    self.cmd_vel.linear.x = 0
                    self.cmd_vel.angular.z = 0.5
                self.pub.publish(self.cmd_vel)
            self.rate.sleep()

    def run(self):
        while not rospy.is_shutdown():
            self.bug0()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        bug0_node = Bug0()
        bug0_node.run()
    except rospy.ROSInterruptException:
        pass
