#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3,Quaternion
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import PoseStamped



class calibrateVectors:
    

    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        self.eThetaVec = 0 #Error between shortest lidar distance and puzzlebot orientation
        self.position = Vector3() #Actual position of the robot
        self.quats = Quaternion()
        self.orientation = 0    #Actual orientation of the robot
        self.lidar_data = []    #Lidar data
        self.obstacle_detected = False

    def euler_from_quaternion(self):
        t3 = +2.0 * (self.quats.w * self.quats.z + self.quats.x * self.quats.y)
        t4 = +1.0 - 2.0 * (self.quats.y * self.quats.y + self.quats.z * self.quats.z)
        self.orientation = math.atan2(t3, t4)

    def getAngle(self, vector_1, vector_2): #Returns the angle between two vectors
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        self.eThetaVec = np.arccos(dot_product)

    def odometry_callback(self, data):
        # Actualización de la posición actual del robot
        self.position = data.pose.pose.position
        self.quats = data.pose.pose.orientation
        self.euler_from_quaternion()
    
    def lidar_callback(self, data):
        # Actualización de la posición actual del robot
        self.lidar_data = data.ranges
    
    def run(self):
        if self.lidar_data.min < 0.5:
            self.obstacle_detected = True
        if self.obstacle_detected == True:
            self.getAngle(self.lidar_data.range_min, self.orientation)
            if self.eThetaVec > 0.1:
                
                #Turn the robot
                pass
            else:
                #Move the robot
                pass

if __main__=='__main__':
    rospy.init_node('calibrateVectors')

    node = calibrateVectors()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

        
    

        