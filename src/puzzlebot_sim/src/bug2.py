#!/usr/bin/env python3
import rospy
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import PoseStamped
from geometry_msgs.msg import Twist, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Bug2Control:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/pose_sim', PoseStamped, self.pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.twist_msg = Twist()
        self.goal_x = rospy.get_param('~goal_x', 5)
        self.goal_y = rospy.get_param('~goal_y', 5)
        self.distance_tolerance = rospy.get_param('~distance_tolerance', 0.5)
        self.rot_speed = rospy.get_param('~rot_speed', 0.5)
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)
        self.covariance_pub = rospy.Publisher('/covariance', PointStamped, queue_size=10)
        self.covariance = [[0, 0], [0, 0]]
        self.current_x = 0
        self.current_y = 0

    def pose_callback(self,data):
        self.current_x = data.pose.position.x
        self.current_y = data.pose.position.x

    def lidar_callback(self, data):
        min_distance = min(data.ranges)
        goal_distance = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)

        if min_distance > self.distance_tolerance and goal_distance > self.distance_tolerance:
            self.twist_msg.linear.x = self.linear_speed
            self.twist_msg.angular.z = 0
        else:
            self.twist_msg.linear.x = 0
            self.twist_msg.angular.z = self.rot_speed

        self.cmd_vel_pub.publish(self.twist_msg)

        # Publica la matriz de covarianza en el tópico /covariance
        covariance_msg = PointStamped()
        covariance_msg.header.stamp = rospy.Time.now()
        covariance_msg.header.frame_id = 'base_link'
        covariance_msg.point.x = self.covariance[0][0]
        covariance_msg.point.y = self.covariance[0][1]
        covariance_msg.point.z = self.covariance[1][1]
        self.covariance_pub.publish(covariance_msg)

    def odom_callback(self, data):
        # Actualiza la matriz de covarianza
        self.current_x = data.pose.pose.position.x
        self.current_y = data.pose.pose.position.y
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.covariance[0][0] = data.pose.covariance[0]
        self.covariance[0][1] = data.pose.covariance[1]
        self.covariance[1][0] = data.pose.covariance[6]
        self.covariance[1][1] = data.pose.covariance[7]

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('bug2_lidar_odom_covariance')
    bug2_control = Bug2Control()
    bug2_control.run()
