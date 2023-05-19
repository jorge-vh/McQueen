#!/usr/bin/env python3
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from tf2_geometry_msgs import PoseStamped
from tf.transformations import euler_from_quaternion

class Bug0Control(object):
    def __init__(self,goal_x,goal_y):
        rospy.init_node('bug0_control')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber('/pose_sim', PoseStamped, self.pose_callback)
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_reached = False
        self.distance_to_goal = 0.0
        self.robot_x = 0
        self.robot_y = 0
        self.robot_yaw = 0
        self.linear_speed = 0.2
        self.angular_speed = 0.0
        self.lidar_data = []
        self.obstacle_detected = False
        self.twist = Twist()
        #self.pConstant = 0.5

    def scan_callback(self,scan):
        self.lidar_data = scan.ranges
        self.distance_to_obstacle = min(self.lidar_data)
        if self.distance_to_obstacle < 0.5:
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False
    
    def pose_callback(self,pose):
        self.robot_x = pose.pose.position.x
        self.robot_y = pose.pose.position.y
        quaternion = (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        self.robot_yaw = euler[2]
        if self.distance_to_goal < 0.1:
            self.goal_reached = True


    def move_towards_goal(self):
        self.distance_to_goal = math.sqrt((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)
        angle_to_goal = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
        angle_difference = angle_to_goal - self.robot_yaw
        if angle_difference > math.pi:
            self.twist.angular.z = -0.1
            self.twist.linear.x = 0.0
        elif angle_difference < -math.pi:
            self.twist.angular.z = 0.1
            self.twist.linear.x = 0.0
        elif angle_difference > 0:
            self.twist.angular.z = 0.1
            self.twist.linear.x = 0.0
        elif angle_difference < 0:
            self.twist.angular.z = -0.1
            self.twist.linear.x = 0.0
        if abs(angle_difference) < 0.05:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0
        if self.distance_to_goal < 0.1:
            self.goal_reached = True
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)
        print(self.twist)
        print(angle_difference)


    def spin(self):
        # Ejecutar el algoritmo Bug0
        self.move_towards_goal()

if __name__ == '__main__':
# Coordenadas de la meta
    goal_x = 1.0
    goal_y = -1.0
    # Crear un objeto Bug0Control
    bug0 = Bug0Control(goal_x, goal_y)

    # Ejecutar el nodo
    try:
        while not bug0.goal_reached:
            bug0.spin()
    except rospy.ROSInterruptException:
        pass
       
            