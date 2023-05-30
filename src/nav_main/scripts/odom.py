#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from math import cos, sin

class pose_estimator:
    def __init__(self):
        # Setup Variables to be used
        self.first = True
        self.start_time=0.0
        self.current_time=0.0
        
        self.x_k = 0.0
        self.y_k = 0.0
        self.theta_k = 0.0
        
        self.odom = Odometry()
        self.odom.pose.pose.position.x = 0
        self.odom.pose.pose.position.y = 0
        self.odom.pose.pose.orientation.w = 0
        
        # Variables for the storage of wheel speeds
        self.wr = 0.0
        self.wl = 0.0
        #Wheel radius and wheelbase
        self.wheel_radius = 0.05
        self.wheelbase = 0.191
        
        #Setup ROS publishers
        self.odom_pub = rospy.Publisher('/odom',Odometry,queue_size=10)

        #Setup ROS Subscribers
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        
    # Callbacks for wheel speeds
    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        self.wr = (2 * linear_vel + angular_vel * self.wheelbase) / (2 * self.wheel_radius)
        self.wl = (2 * linear_vel - angular_vel * self.wheelbase) / (2 * self.wheel_radius)


        
    def run(self):
        #Run this code only at the beginning
        if (self.first):
            #Wait for the first time to be published from gazebo
            while not rospy.get_time():
                pass
            #Initialise variables to use the first iteration before entering the loop
            self.current_time=rospy.get_time()
            self.start_time=rospy.get_time()
            self.first=False
            self.x_k = 0.0
            self.y_k = 0.0
            self.theta_k = 0.0
            
        else:
            #Calculate time
            self.current_time=rospy.get_time()
            dt=self.current_time-self.start_time
            self.start_time = self.current_time
            
            # Calculate the linear and angular velocities
            linear_vel = (self.wr + self.wl) * self.wheel_radius / 2.0
            angular_vel = (self.wr - self.wl) * self.wheel_radius / self.wheelbase
            
            # Update the robot's position and orientation
            self.x_k += linear_vel * cos(self.theta_k) * dt
            self.y_k += linear_vel * sin(self.theta_k) * dt
            self.theta_k += angular_vel * dt

            #Publish the control inputs
            self.odom.header.stamp = rospy.Time.now()
            self.odom.header.frame_id = 'odom'
            self.odom.child_frame_id = 'base_link'

            # Set position    
            self.odom.pose.pose.position.x = self.x_k
            self.odom.pose.pose.position.y= self.y_k
            self.odom.pose.pose.position.z = 0.0
            self.odom.pose.pose.orientation.w = self.theta_k
            
            self.odom_pub.publish(self.odom)
    
    #Stop Condition
    def stop(self):
        print("Stopping")
        self.odom.pose.pose.position.x = 0
        self.odom.pose.pose.position.y = 0
        self.odom.pose.pose.orientation.w = 0
        self.odom_pub.publish(self.odom)

if __name__ == '__main__':
    #Initialise and Setup node
    rospy.init_node("Pose_estimator")
    RobotCtrl = pose_estimator()
    loop_rate = rospy.Rate(10)
    rospy.on_shutdown(RobotCtrl.stop)

    #Run node
    print("Running")    
    try:
        while not rospy.is_shutdown():
            RobotCtrl.run()
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        pass