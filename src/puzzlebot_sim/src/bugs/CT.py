#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf import TransformBroadcaster

class CoordinateTransform:
    
    def __init__(self):

        # ______________ init node publishers, subscribers and services ______________
        rospy.init_node('coordinate_transform')
        self.transform_broadcaster = TransformBroadcaster()
        self.puzzlebot_pose_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.puzzlebot_pose_x = None
        self.puzzlebot_pose_y = None
        self.puzzlebot_pose_theta = None   

        self.rate = rospy.Rate(10.0)  
        # ______________ end of the init _____________________________________________

        # ______________ callback from the odom topic to fill ________________________
    def odom_callback(self, msg):
        self.puzzlebot_pose_x = msg.pose.pose.position.x
        self.puzzlebot_pose_y = msg.pose.pose.position.y
        _, _, self.puzzlebot_pose_theta = euler_from_quaternion(
            [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        )

        # ______________ end of the callback _________________________________________

    def main(self):
        while not rospy.is_shutdown():
            if self.puzzlebot_pose_x is not None and self.puzzlebot_pose_y is not None and self.puzzlebot_pose_theta is not None:
                # ______________ send transform from one pose to another _____________
                self.transform_broadcaster.sendTransform((self.puzzlebot_pose_x, self.puzzlebot_pose_y, 0.0),
                                quaternion_from_euler(0, 0, self.puzzlebot_pose_theta),
                                rospy.Time.now(),
                                "base_link",
                                "map")                                                
            self.rate.sleep()
                # ______________ end of the main function _____________________________

if __name__ == '__main__':
    instance = CoordinateTransform()
    instance.main()