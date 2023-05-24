#!/usr/bin/env python3


import math
import rospy
import numpy as np
import nav_functions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64MultiArray



class KalmanFilterForPuzzlebotPose():
    def __init__(self):
        
        rospy.init_node("kalman_filter_for_puzzlebot_pose")
        rospy.Subscriber("/kalman_prediction_odom", Odometry, self.odom_callback)                
        rospy.Subscriber("/estimated_sensor_reading", Float64MultiArray, self.estimated_visual_sensor_reading_callback)        
        rospy.Subscriber("/relation_matrix_between_sensor_and_state", Float64MultiArray, self.visual_sensor_reading_c_mat_callback)
        rospy.Subscriber("/real_sensor_reading", Float64MultiArray, self.real_sensor_Reading_callback)

        self.kalman_corrected_odom_pub = rospy.Publisher("/kalman_corrected_odom",Odometry, queue_size=1)
        self.kalman_position_message = Odometry()
        self.puzzlebot_6_times_6_covariance_matrix = np.zeros((6*6,), dtype=float)        

        self.relation_matrix_between_output_and_state = None
        self.state_uncertainty_matrix = np.identity(2) *0.2

        self.predicted_state = None        
        self.predicted_state_covariance = None        
        self.visual_sensor_reading = None

        self.real_yk = None

        self.last_visual_sensor_msg_timestamp = None
        self.last_visual_sensor_processed_msg_timestamp = None
        self.new_sensor_reading_recieved = False
        self.new_relation_matrix_between_state_and_sensor_recieved = False

        self.rate_val = 5.0
        self.rate = rospy.Rate(self.rate_val)    

    def odom_callback(self, data):
        self.odom_msg_to_state_vector(data)
   
    def estimated_visual_sensor_reading_callback(self, msg):                
        reading_as_array = np.array(msg.data)
        self.visual_sensor_reading = reading_as_array.reshape((2,1))
        self.last_visual_sensor_msg_timestamp = rospy.get_time()
        self.new_sensor_reading_recieved = True
    
    def real_sensor_Reading_callback(self, msg):
        reading_as_array = np.array(msg.data)
        self.real_yk = reading_as_array.reshape((2,1))

    def visual_sensor_reading_c_mat_callback(self, msg):
        reading_as_array = np.array(msg.data)
        self.relation_matrix_between_output_and_state = reading_as_array.reshape((2,3))
        self.last_visual_sensor_msg_timestamp = rospy.get_time()
        self.new_relation_matrix_between_state_and_sensor_recieved = True        

    def odom_msg_to_state_vector(self, msg):
        _, _, robot_yaw = nav_functions.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.predicted_state = np.array(
            [[msg.pose.pose.position.x],
             [msg.pose.pose.position.y],
             [robot_yaw]]
        )
        self.predicted_state_covariance = np.array(
            [[msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[5] ],
             [msg.pose.covariance[6], msg.pose.covariance[7], msg.pose.covariance[11]],
             [msg.pose.covariance[30], msg.pose.covariance[31], msg.pose.covariance[35]]],
        )

    def kalman(self,xk_pred,Pk_pred,C,R,yk):        
        #Correction
        Gk = Pk_pred@(C.T)@np.linalg.inv((C@Pk_pred@C.T)+R)        
        xk_corrected = xk_pred+ Gk@(self.real_yk - yk) 
        Pk_corrected = (np.identity(len(xk_pred), dtype=float)-(Gk@C))@Pk_pred        
        return xk_corrected,Pk_corrected
    
    def fill_odometry_header(self):
        self.kalman_position_message.header.stamp = rospy.Time.now()
        self.kalman_position_message.header.frame_id = "map"
        self.kalman_position_message.child_frame_id = "base_link"        
    
    def create_kalman_position_message(self,corrected_state_mean, corrected_state_cov):
        
        self.fill_odometry_header()
        self.kalman_position_message.pose.pose.position.x = corrected_state_mean[0,0]
        self.kalman_position_message.pose.pose.position.y = corrected_state_mean[1,0]

        self.puzzlebot_estimated_rot_quaternion = nav_functions.quaternion_from_euler(0.0, 0.0, corrected_state_mean[2,0])
        self.kalman_position_message.pose.pose.orientation.x = self.puzzlebot_estimated_rot_quaternion[0]
        self.kalman_position_message.pose.pose.orientation.y = self.puzzlebot_estimated_rot_quaternion[1]
        self.kalman_position_message.pose.pose.orientation.z = self.puzzlebot_estimated_rot_quaternion[2]
        self.kalman_position_message.pose.pose.orientation.w = self.puzzlebot_estimated_rot_quaternion[3]
                
        self.puzzlebot_6_times_6_covariance_matrix[0:2] = corrected_state_cov[0,0:2]
        self.puzzlebot_6_times_6_covariance_matrix[5] = corrected_state_cov[0,2]
        self.puzzlebot_6_times_6_covariance_matrix[6:8] = corrected_state_cov[1,0:2]
        self.puzzlebot_6_times_6_covariance_matrix[11] = corrected_state_cov[1,2]
        self.puzzlebot_6_times_6_covariance_matrix[30:32] = corrected_state_cov[2,0:2]
        self.puzzlebot_6_times_6_covariance_matrix[35] = corrected_state_cov[2,2]

        self.kalman_position_message.pose.covariance = self.puzzlebot_6_times_6_covariance_matrix.tolist()
        
        self.kalman_corrected_odom_pub.publish(self.kalman_position_message)        
                
    def main(self):        
        while not rospy.is_shutdown():
            if self.predicted_state is not None and self.predicted_state_covariance is not None:
                if self.last_visual_sensor_msg_timestamp == None and self.last_visual_sensor_processed_msg_timestamp == None:
                    
                    corrected_state_mean = self.predicted_state
                    corrected_state_cov = self.predicted_state_covariance
                
                elif (((self.last_visual_sensor_processed_msg_timestamp == None and self.last_visual_sensor_msg_timestamp != None) or 
                (self.last_visual_sensor_processed_msg_timestamp < self.last_visual_sensor_msg_timestamp)) and
                (self.new_sensor_reading_recieved and self.new_relation_matrix_between_state_and_sensor_recieved)):
                    
                    # TODO Check elif conditions
                    
                    corrected_state_mean, corrected_state_cov = self.kalman(self.predicted_state,
                                                                            self.predicted_state_covariance,
                                                                            self.relation_matrix_between_output_and_state,
                                                                            self.state_uncertainty_matrix,
                                                                            self.visual_sensor_reading)
                    self.last_visual_sensor_processed_msg_timestamp = self.last_visual_sensor_msg_timestamp
                    if np.any(np.isnan(corrected_state_mean)):
                        corrected_state_mean = self.predicted_state
                        corrected_state_cov = self.predicted_state_covariance
                                    
                else:
                    
                    corrected_state_mean = self.predicted_state
                    corrected_state_cov = self.predicted_state_covariance    

  
                self.create_kalman_position_message(corrected_state_mean, corrected_state_cov) 
            
            else:
                print ("yk is none")               
            self.rate.sleep()          

if __name__ == "__main__":
    kalman_filer = KalmanFilterForPuzzlebotPose()
    kalman_filer.main()
