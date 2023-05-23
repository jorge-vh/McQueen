#!/usr/bin/env python3

#Last Modify: Fernando Cuellar 
import math
import rospy
import nav_functions
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Bug0():
    def __init__(self, targetx, targety, wall_distance):
        # ______________ init node publishers, subscribers and services ______________
        rospy.init_node("bug_0_controller")
        rospy.Subscriber("/odom", Odometry, self.odom_callback)                
        
        self.scan_listener = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)        
        self.vel_msg = Twist()
        # ______________ init end ____________________________________________________

        self.target_postition_xy_2d = (targetx, targety)
        self.wall_distance = wall_distance
        print("Target is :", self.target_postition_xy_2d)
                
        self.current_position_xy_2d = (None, None)
        self.scan = None
        self.current_angle = None
        self.displaced_angle = 0.0
                     
        # ______________ creation of values and constants_____________________________             
        self.angular_error_treshold = 0.3    
        self.distance_error_treshold = 0.08                    

        self.go2point_angular_kp = 0.08
        self.go2point_linear_kp = 0.3

        self.wall_kp_follow = 40.0
        self.wall_kp_avoid = 0.3

        self.v_max = 0.4
        self.v_max_wall = 0.4
        self.w_max = 0.6

        self.puzzlebot_passing_diameter = 0.60 # meters
        # ____________________________________________________________________________ 

        self.state = "go_to_point"

        self.turn_left_go_to_point_state_counter = 0
        self.follow_wall_start_time = None       

        self.rate_val = 20.0
        self.rate = rospy.Rate(self.rate_val)        

        self.last_turn_time = None

        # ______________ callback for the scan_______________________________________ 
    def scan_callback(self, msg):
        self.scan = msg    

        # ______________ function to reset values____________________________________
    def reset_values(self):      
        self.vel_msg = Twist()        
        self.current_position_xy_2d = (None, None)
        self.target_postition_xy_2d = (None, None)
        self.current_angle = None        
        self.displaced_angle = 0.0
        # ______________ callback for the odom topic_________________________________
    def odom_callback(self, data):
        #vector to use the position in x and y
        self.current_position_xy_2d = ( data.pose.pose.position.x , data.pose.pose.position.y )
        #obtain the angle from the odom parameter to degrees                              
        self.current_angle = nav_functions.calculate_yaw_angle_deg( data.pose.pose.orientation )

        # ______________ function to  turn left in case of state_____________________
    def turn_left(self, p2p_target_angle, angle_to_rotate = np.pi/3.0):
        #turn left until angle to rotate has been complete 
        if self.last_turn_time == None and self.state == "turn_left":
            self.last_turn_time = rospy.get_time()
        else:    
            if self.displaced_angle < angle_to_rotate: #angle_to_rotate = np.pi/2.0                
                linear_x = 0.0
                angular_z = self.w_max            
                self.vel_msg.linear.x = linear_x
                self.vel_msg.angular.z = angular_z                
                current_time = rospy.get_time()
                self.displaced_angle += abs(angular_z)*(current_time - self.last_turn_time) #sigma of deltas 
                self.last_turn_time = current_time
            else:
                self.state = "follow_wall" #one of the states 
                self.follow_wall_start_time = rospy.get_time()                
                self.displaced_angle = 0.0        
                self.last_turn_time = None            

    def go_to_point_controller(self, angle_error, distance_error):        
        front_scan_val = self.get_mean_laser_value_at_fov(0.0, 30.0)        
        if distance_error <= self.distance_error_treshold: #get close to error distance                                                            
            self.state = "arrived" #one of the states 
        elif self.obstacle_in_front(): #is there is an obstacle go to state turn left 
                self.state = "turn_left"
        elif abs(angle_error) > self.angular_error_treshold: #redirect the angle bi the kp and the angle error                                    
            self.vel_msg.angular.z = nav_functions.saturate_signal(self.go2point_angular_kp*angle_error, self.w_max) 
        elif distance_error > self.distance_error_treshold:              
            self.vel_msg.linear.x = nav_functions.saturate_signal(self.go2point_linear_kp*distance_error, self.v_max)    

#         def saturate_signal(signal, saturation_value):
#        if signal > abs(saturation_value):
#            result = abs(saturation_value)
#        elif signal < -abs(saturation_value):
#            result = -abs(saturation_value)
#        else:
#            result = signal
#        return result  """   

        # ______________ obtain the index in with the beam is crashing________________________
    def get_laser_index_from_angle(self, angle_in_deg):        
        angle_index = round( (angle_in_deg*len(self.scan.ranges))/360.0 )
        return int(angle_index)
    
    def get_mean_laser_value_at_fov(self, fov_center, fov_range):        
        if fov_range > 180.0:
            Exception("fov is too large") #can handle more degrees for this case 
        if fov_center - (fov_range/2.0) < 0.0:
            lower_boundary = nav_functions.angle_to_only_possitive_deg(fov_center - (fov_range/2.0))            
            values_at_fov_1 = self.scan.ranges[self.get_laser_index_from_angle(lower_boundary) : ]
            values_at_fov_2 = self.scan.ranges[: self.get_laser_index_from_angle( fov_center + (fov_range/2.0) ) + 1]
            values_at_fov = np.array(values_at_fov_1 + values_at_fov_2 )
        elif fov_center + (fov_range/2.0) > 360.0:
            values_at_fov_1 = self.scan.ranges[self.get_laser_index_from_angle(fov_center - (fov_range/2.0)) : ]
            values_at_fov_2 = self.scan.ranges[: self.get_laser_index_from_angle( (fov_center + (fov_range/2.0))%360.0 ) + 1]
            values_at_fov = np.array(values_at_fov_1 + values_at_fov_2 )
        else:
            values_at_fov = np.array( self.scan.ranges[ self.get_laser_index_from_angle(fov_center-(fov_range/2.0)) : self.get_laser_index_from_angle(fov_center+(fov_range/2.0)) + 1 ] )        
        
        values_at_fov[values_at_fov == np.inf] = 12.0
        return values_at_fov.mean()/1.2 # since values do not appear to represent the real meters
    
    def get_values_at_target(self, fov_center, fov_range):        
        if fov_range > 180.0:
            Exception("fov is too large")
        if fov_center - (fov_range/2.0) < 0.0:
            values_at_fov_1 = self.scan.ranges[self.get_laser_index_from_angle(nav_functions.angle_to_only_possitive_deg(fov_center - (fov_range/2.0))) : ]
            values_at_fov_2 = self.scan.ranges[: self.get_laser_index_from_angle( fov_center + (fov_range/2.0) ) + 1]
            values_at_fov = values_at_fov_1 + values_at_fov_2
        elif fov_center + (fov_range/2.0) > 360.0:
            values_at_fov_1 = self.scan.ranges[self.get_laser_index_from_angle(fov_center - (fov_range/2.0)) : ]
            values_at_fov_2 = self.scan.ranges[: self.get_laser_index_from_angle( (fov_center + (fov_range/2.0))%360.0 ) + 1]
            values_at_fov = values_at_fov_1 + values_at_fov_2
        else:
            values_at_fov = self.scan.ranges[ self.get_laser_index_from_angle(fov_center-(fov_range/2.0)) : self.get_laser_index_from_angle(fov_center+(fov_range/2.0)) + 1 ]
        
        return values_at_fov
    
    def target_path_is_clear(self, p2p_target_angle):
        #check if in the range of the wall distance and its angle 
        wall_dist_fov = 2.0*( np.pi/2.0 - np.arctan(self.wall_distance/(self.puzzlebot_passing_diameter/2)) ) 
        #calculate the target angle direction base on our current angle       
        target_direction = nav_functions.angle_to_only_possitive_deg(p2p_target_angle - self.current_angle)        
        values_at_target = self.get_values_at_target(target_direction, wall_dist_fov)        
        target_is_clear = ( (min(values_at_target) >= 2.25*self.wall_distance) and 
        ((target_direction > 0.0 and target_direction < 90.0) or 
        ((target_direction > 270.0) and target_direction < 360.0)) )

        return target_is_clear
        
        # ______________ falg to know if there is an obstacle ________________________
    def obstacle_in_front(self):
        wall_dist_fov = 2.0*( np.pi/2.0 - np.arctan(self.wall_distance/(self.puzzlebot_passing_diameter/2)) )                
        values_in_front = self.get_values_at_target(0.0, wall_dist_fov)
        side_dists_to_obstacle = self.get_values_at_target(315.0, 30.0)
        #if theres a min value than the wall distance or to the sides 
        return min(values_in_front) <= self.wall_distance or min(side_dists_to_obstacle) <= self.wall_distance/2.0
    
    def right_hand_rule_controller(self, p2p_target_angle):
        if self.scan != None:                       
            if self.target_path_is_clear(p2p_target_angle) and (rospy.get_time() - self.follow_wall_start_time) >= 1.0:
                self.state = "go_to_point"
            elif self.obstacle_in_front():
                self.state = "turn_left"                
            else:
                # TODO complete linear_x and angular_z vals                
                dist_at_0 = self.scan.ranges[self.get_laser_index_from_angle(270.0)]
                dist_at_10 = self.scan.ranges[self.get_laser_index_from_angle(280.0)]
                l3 = np.sqrt( (dist_at_0**2.0) + (dist_at_10**2.0) - 2.0*dist_at_0*dist_at_10*(np.cos( 0.174533 )) )
                y1 = np.arcsin((dist_at_10*( np.sin(0.174533) ))/l3)
                ang_err = np.pi/2.0 - y1                            
                dist_to_wall = min( self.get_values_at_target(270.0, 90.0) ) #right side 
                if dist_to_wall == np.inf:
                    dist_to_wall = 12.0
                #dist_to_wall = dist_at_0
                relative_euclidean_distance_to_wall = self.wall_distance - dist_to_wall

                angular_z = nav_functions.saturate_signal( self.wall_kp_avoid*relative_euclidean_distance_to_wall, self.w_max )
                if self.wall_kp_follow*ang_err != 0:
                    linear_x = nav_functions.saturate_signal( 1/(self.wall_kp_follow* abs(ang_err)), self.v_max_wall )
                    if linear_x <= self.v_max_wall/2.0:
                        linear_x = self.v_max_wall/2.0
                else:
                    linear_x = self.v_max_wall

                if dist_at_0 >= 1.2 or ang_err < (-np.pi/2.0)*(1.0/3.0):
                    angular_z = -(self.v_max_wall/self.wall_distance)
                    linear_x = self.v_max_wall                            

                self.vel_msg.angular.z = angular_z
                self.vel_msg.linear.x = linear_x
                
    def main(self):
        print("main inited node running")
        while not rospy.is_shutdown():
            print(self.state)         
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = 0.0
            #wait to start 
            if self.current_angle != None and self.scan != None:
                #fill the vector of target position - current position 
                target_vector_minus_robot_vector = ( self.target_postition_xy_2d[0] - self.current_position_xy_2d[0],
                                                     self.target_postition_xy_2d[1] - self.current_position_xy_2d[1]  )
                #get target angle from the previous vector, (atan2, from the distance between the current and target)  
                target_angle = nav_functions.rad2deg(math.atan2( target_vector_minus_robot_vector[1],target_vector_minus_robot_vector[0]))   
                #get angle error in degrees from 0 to 360             
                angle_error = nav_functions.calculate_angular_error_considering_upper_boundary_lag(target_angle, self.current_angle, "deg", (0.0,360.0))      
                #get distance error from current position to target           
                distance_error = nav_functions.euclidean_distance_single_point_2d( target_vector_minus_robot_vector )            
                # ______________ states machine  ________________________             
                if self.state == "go_to_point":
                    self.go_to_point_controller(angle_error, distance_error) #calculate angle error, give direction and go
                elif self.state == "follow_wall":
                    self.right_hand_rule_controller(target_angle)  
                elif self.state == "turn_left":
                    self.turn_left(target_angle) 
                elif self.state == "arrived":
                    print("You have arrived to the destination")
                    return        
                self.vel_pub.publish(self.vel_msg)
            self.rate.sleep()          

if __name__ == "__main__":
    bug_0 = Bug0(5.0,-0.6,0.5) # target coordinates  
    bug_0.main()
    bug_0 = Bug0(8.0,2.0,0.5) # target coordinates  
    bug_0.main()
    bug_0 = Bug0(3.0,2.0,0.5) # target coordinates  
    bug_0.main()
    bug_0 = Bug0(0.0,0.0,0.5) # target coordinates  
    bug_0.main()
    
    
