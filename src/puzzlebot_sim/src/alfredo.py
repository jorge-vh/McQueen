#!/usr/bin/env python3
import csv, time, math

import rospy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor

from tf.transformations import euler_from_quaternion

from .bug_0 import *

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from tf2_geometry_msgs import PoseStamped

TOPIC_CALCULATED_POSE = '/calculated_pose'
TOPIC_LIDAR_SCAN = '/scan'
TOPIC_VEL_CMD = '/cmd_vel'

class PuzzlebotBug(Node):
  def __init__(self):
    super().__init__('puzzlebot_bug_zero')
    self.get_logger().info('init...')
    self.position = np.array([0, 0])
    self.orientation = 0

    self.goal = np.array([0, 2])
    self.lidar_data = np.array([])

    self.bug = BugZero(30)
    self.bug.set_goal(self.goal)

    self.create_subscription(
        LaserScan, TOPIC_LIDAR_SCAN, self.set_lidar_data, 10)
    self.create_subscription(
        PoseStamped, TOPIC_CALCULATED_POSE, self.set_position, 10)
    self.pub_cmd_vel = self.create_publisher(
        Twist, TOPIC_VEL_CMD, 10)


  def set_lidar_data(self, msg: LaserScan):
    ranges = msg.ranges
    l = len(msg.ranges)
    vectors = []
    for idx, d in enumerate(ranges):
      deg = (idx / l)*360
      # Lidar angle 0 is puzzlebot angle 180
      rad = np.deg2rad(deg + 180)
      vectors.append([d*sin(rad), d*cos(rad)])
    self.lidar_data = np.array(vectors)
  

  def set_position(self, msg: PoseStamped):
    self.position = np.array([msg.pose.position.x, msg.pose.position.y])
    orientation = msg.pose.orientation
    quats = [orientation.x, orientation.y, orientation.z, orientation.w]
    eulers = euler_from_quaternion(quats)
    self.orientation = eulers[2]


  def publish_direction(self, direction):
    direction = unit_vector(direction)
    angle_error = math.atan2(direction[1], direction[0]) - self.orientation
    twist_msg = Twist()
    self.get_logger().info(
      'angle error: %.2f' % angle_error)
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
    self.get_logger().info('goal error: %.2f' % err)
    return err < 0.1


  def get_lidar_data(self):
    res = []
    for d in self.lidar_data:
      if d[0] != float('inf') and d[1] != float('inf'):
        res.append(d)
    return np.array(res)


  def run(self):
    rate = self.create_rate(50)
    while not self.on_goal():
      rate.sleep()
      if len(self.lidar_data) == 0:
        continue
      step, circumnavigating = self.bug.next_step(
          self.position, self.lidar_data)
      self.publish_direction(step)
    self.stop()
    self.stop()


def main(args = None):
    rclpy.init(args = args)
    executor = MultiThreadedExecutor()
    node = PuzzlebotBug()
    executor.add_node(node)
    task = executor.create_task(node.run)
    try:
        executor.spin_until_future_complete(task)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()    
