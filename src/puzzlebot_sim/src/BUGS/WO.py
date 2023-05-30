#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Pose2D, TransformStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf2_msgs.msg import TFMessage


class WheelOdometry():
    
    def __init__(self, initial_x = 0.0, initial_y = 0.0, initial_theta = 0.0):
        
        # ______________ init node publishers, subscribers and services ______________
        rospy.init_node('wheel_odometry')        
        self.wr_sub = rospy.Subscriber('/wr', Float32, self.puzzlebot_wr_callback)
        self.wl_sub = rospy.Subscriber('/wl', Float32, self.puzzlebot_wl_callback)        
        self.puzzlebot_odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.puzzlebot_tfPub = rospy.Publisher('/tfPB',TFMessage,queue_size=1)
        self.puzzlebot_tf = TFMessage()
        self.transformPbLidar = TransformStamped()
        self.lidarOrient = quaternion_from_euler(0, 0, 1)
        
        # ______________ fill in publisher messages ______________
        self.initial_x, self.initial_y, self.initial_theta = (initial_x, initial_y, initial_theta)        
        
        self.puzzlebot_estimated_pose = Odometry()        
        self.puzzlebot_estimated_pose.pose.pose.position.x = self.initial_x
        self.puzzlebot_estimated_pose.pose.pose.position.y = self.initial_y    
        self.puzzlebot_estimated_rot = self.initial_theta   

        self.first_time_with_w = True     
        
        # ______________ init nav variables ______________
        self.puzzlebot_estimated_rot_quaternion = None
        self.last_sampling_time = None        
        self.puzzlebot_wheel_rad = 0.057 # radius of the wheel
        self.puzzlebot_wheel_to_wheel_dist = 0.192 # length of the arrow
        self.puzzlebot_wr = None
        self.puzzlebot_wl = None
        self.v = 0.0
        self.w = 0.0

        # ______________ init rate ______________
        self.rate = rospy.Rate(10.0)            


    def puzzlebot_wr_callback(self, msg):
        self.puzzlebot_wr = msg.data        

    def puzzlebot_wl_callback(self, msg):
        self.puzzlebot_wl = msg.data         

    def fill_odometry_header(self):
        self.puzzlebot_estimated_pose.header.stamp = rospy.Time.now()
        self.puzzlebot_estimated_pose.header.frame_id = "world"
        self.puzzlebot_estimated_pose.child_frame_id = "base_link"

    def main(self):
        while not rospy.is_shutdown():
            if  self.puzzlebot_wr is not None and self.puzzlebot_wl is not None:
                if self.first_time_with_w:
                    self.first_time_with_w = False
                    self.last_sampling_time = rospy.get_time()                    
                    rospy.loginfo("Odometry initialized")
                else:
                    current_time = rospy.get_time()
                    delta_t = (current_time - self.last_sampling_time)
                    self.fill_odometry_header()
                    
                    # _________ wheels' odometry ______________
                    self.v = (self.puzzlebot_wheel_rad/2)*self.puzzlebot_wr + (self.puzzlebot_wheel_rad/2)*self.puzzlebot_wl
                    self.w = (self.puzzlebot_wheel_rad/self.puzzlebot_wheel_to_wheel_dist)*self.puzzlebot_wr - (self.puzzlebot_wheel_rad/self.puzzlebot_wheel_to_wheel_dist)*self.puzzlebot_wl
                    # _________ end of wheels' odometry ______________


                    # _________ filling puzzlebot state data ______________
                    self.puzzlebot_estimated_pose.pose.pose.position.x += self.v*(np.cos(self.puzzlebot_estimated_rot))*delta_t
                    self.puzzlebot_estimated_pose.pose.pose.position.y += self.v*(np.sin(self.puzzlebot_estimated_rot))*delta_t
                    self.puzzlebot_estimated_rot += self.w*delta_t
                    self.puzzlebot_estimated_rot_quaternion = quaternion_from_euler(0.0, 0.0, self.puzzlebot_estimated_rot)
                    self.puzzlebot_estimated_pose.pose.pose.orientation.x = self.puzzlebot_estimated_rot_quaternion[0]
                    self.puzzlebot_estimated_pose.pose.pose.orientation.y = self.puzzlebot_estimated_rot_quaternion[1]
                    self.puzzlebot_estimated_pose.pose.pose.orientation.z = self.puzzlebot_estimated_rot_quaternion[2]
                    self.puzzlebot_estimated_pose.pose.pose.orientation.w = self.puzzlebot_estimated_rot_quaternion[3]

                    self.transformPbLidar.header.stamp = rospy.Time.now()
                    self.transformPbLidar.header.frame_id = 'base_link'
                    self.transformPbLidar.child_frame_id = 'laser'
                    self.transformPbLidar.transform.translation.z = 0.2
                    self.transformPbLidar.transform.rotation.x = self.lidarOrient[0]
                    self.transformPbLidar.transform.rotation.y = self.lidarOrient[1]
                    self.transformPbLidar.transform.rotation.z = self.lidarOrient[2]
                    self.transformPbLidar.transform.rotation.w = self.lidarOrient[3]
                    self.puzzlebot_tf = [self.transformPbLidar]
                    # _________ end of filling puzzlebot pose data ______________

                    self.last_sampling_time = current_time                    
            self.puzzlebot_odom_pub.publish(self.puzzlebot_estimated_pose)
            self.puzzlebot_tfPub.publish(self.puzzlebot_tf)
            self.rate.sleep()

if __name__ == '__main__':
    instance = WheelOdometry()
    instance.main()
