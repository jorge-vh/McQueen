#!/usr/bin/env python3
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3,Quaternion,PoseStamped
from std_msgs.msg import String
import math

class bug0Vectors:
    def __init__(self):
        self.lidar_data = []
        self.position = Vector3()
        self.orientation = 0
        self.quats = Quaternion()
        self.eTheta = 0
        self.vecGoal = [0,0] #Vector from robot to goal
        self.vecOrient = [0,0] #Actual orientation of the robot
        self.vecObs = [0,0] #Vector from robot to obstacle
        self.vecPer = [0,0] #Vector perpendicular to vecObs
    
    def lidar_callback(self, data):
        self.lidar_data = ros_numpy.numpify(data.ranges)
    
    def pose_callback(self, data):
        self.position = data.pose.position
        self.quats = data.pose.orientation
        self.orientation = self.quats.z
        self.eTheta = math.atan2(2*(self.quats.w*self.quats.z+self.quats.x*self.quats.y),1-2*(self.quats.y**2+self.quats.z**2))
        self.vecOrient = [math.cos(self.eTheta),math.sin(self.eTheta)]
    
    def calculateError(self):

    