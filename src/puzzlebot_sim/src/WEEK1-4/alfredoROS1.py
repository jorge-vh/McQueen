#!/usr/bin/env python3

import rospy
import math
import numpy as np
import tf
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from tf2_geometry_msgs import PoseStamped

from bug_0 import BugZero

TOPIC_CALCULATED_POSE = '/pose_sim'
TOPIC_LIDAR_SCAN = '/scan'
TOPIC_VEL_CMD = '/cmd_vel'

class PuzzlebotBug:

    def __init__(self):
        self.position = np.array([0, 0])
        self.orientation = 0

        self.goal = np.array([0, 2])
        self.lidar_data = np.array([])

        self.bug = BugZero(30)
        self.bug.set_goal(self.goal)

        rospy.Subscriber(TOPIC_LIDAR_SCAN, LaserScan, self.set_lidar_data)
        rospy.Subscriber(TOPIC_CALCULATED_POSE, PoseStamped, self.set_position)
        self.pub_cmd_vel = rospy.Publisher(TOPIC_VEL_CMD, Twist, queue_size=10)

    def unit_vector(self,vector):
        return vector / np.linalg.norm(vector)

    def set_lidar_data(self, msg):
        ranges = msg.ranges
        l = len(msg.ranges)
        vectors = []
        for idx, d in enumerate(ranges):
            deg = (idx / l)*360
            # Lidar angle 0 is puzzlebot angle 180
            rad = np.deg2rad(deg + 180)
            vectors.append([d*np.sin(rad), d*np.cos(rad)])
        self.lidar_data = np.array(vectors)

    def set_position(self, msg):
        self.position = np.array([msg.pose.position.x, msg.pose.position.y])
        orientation = msg.pose.orientation
        quats = [orientation.x, orientation.y, orientation.z, orientation.w]
        eulers = euler_from_quaternion(quats)
        self.orientation = eulers[2]

    def publish_direction(self, direction):
        direction = self.unit_vector(direction)
        angle_error = math.atan2(direction[1], direction[0]) - self.orientation
        twist_msg = Twist()
        rospy.loginfo('angle error: %.2f' % angle_error)
        if abs(angle_error) < np.pi/55:
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = 0.0
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = min(0.1, 0.15*angle_error)
        self.pub_cmd_vel.publish(twist_msg)

    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(twist_msg)

    def on_goal(self):
        err = np.linalg.norm(self.goal - self.position)
        rospy.loginfo('goal error: %.2f' % err)
        return err < 0.1

    def get_lidar_data(self):
        res = []
        for d in self.lidar_data:
            if d[0] != float('inf') and d[1] != float('inf'):
                res.append(d)
        return np.array(res)

    def run(self):
      rate = rospy.Rate(50)

      while not self.on_goal() and not rospy.is_shutdown():
        rate.sleep()

        if len(self.lidar_data) == 0:
          continue

        step, circumnavigating = self.bug.next_step(
            self.position, self.get_lidar_data())

        self.publish_direction(step)

      self.stop()
      self.stop()

def main(args = None):
    rospy.init_node('bug',anonymous=True)
    node = PuzzlebotBug()   
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':
    main()    
