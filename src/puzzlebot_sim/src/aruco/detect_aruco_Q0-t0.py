#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Int8, Header
from nav_functions import *


class ArucoDetector():
    def __init__(self, aruco_dict = cv2.aruco.DICT_4X4_50):
        rospy.init_node("aruco_detector")
        # ________ aruco atributes initialization ______
        self.arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        self.arucoCoordinates = {"0": (2,2),"1": (2,-2),"2": (-2,-2),"3": (-2,2),"4": (4,4),"5": (4,-4),"6": (-4,-4),"7": (-4,4)}
        self.arucoBoxDim = 0.24

        # ________ ros atributes initialization ______        
        self.image_pub = rospy.Publisher("/image_detecting", Image, queue_size = 1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan,self.scan_callback)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        #__________ image ______________
        self.curr_signs_image_msg = Image()
        self.curr_signs_image_msg_2 = Image()
        self.curr_signs_image_msg_3 = Image()

        #___________ color ______________
        self.green = (0, 255, 0)

        #___________ video initialization _______________
        self.displayed_image_ocv = np.zeros(5, dtype=np.uint8)
        self.image = None
        self.ocv_image = None

        self.scan = None

        self.current_position_xy_2d = None
        self.current_angle = None

        self.rate = rospy.Rate(5.0)        

    def image_callback(self, msg):
        self.image = msg
        self.main()

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, data):
        self.current_position_xy_2d = ( data.pose.pose.position.x , data.pose.pose.position.y )                             
        _, _, yaw = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.current_angle = yaw*(180.0/math.pi)
        #print("current angle {c}".format(c = self.current_angle))

    def draw_arucos(self, image, corners):
        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
			# loop over the detected ArUCo corners
            for markerCorner in corners:
				# extract the marker corners (which are always returned
				# in top-left, top-right, bottom-right, and bottom-left
				# order)
                corners = markerCorner.reshape((4, 2))

                (topLeft, topRight, bottomRight, bottomLeft) = corners

				# convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

				# draw the bounding box of the ArUCo detection
                image = cv2.line(image, topLeft, topRight, self.green, 2)
                image = cv2.line(image, topRight, bottomRight, self.green, 2)
                image = cv2.line(image, bottomRight, bottomLeft, self.green, 2)
                image = cv2.line(image, bottomLeft, topLeft, self.green, 2)
        return image

    def get_arucos_info_in_image(self, image):
        # detect ArUco markers in the input frame
        (corners, ids, rejected) = self.arucoDetector.detectMarkers(image)    
        return (corners, ids)

    def get_aruco_angle(self, aruco_midpoint, camera_fov = 1.3962634, image_width = 800.0):
        """
        input:
            aruco_midpoint : tuple with (x_center_px, y_center_px)
        output:
            aruco_angle: float with aruco angle in rads
        """
        angle = -(camera_fov/image_width)*aruco_midpoint[0] + camera_fov/2
        return angle

    def midpoint_equation(self, p1, p2):
        return ( (p1[0]+p2[0])/2, (p1[1]+p2[1])/2 )

    def get_aruco_midpoint(self, rectangle_corners):
        """ function that returns the x,y cordinates of the aruco's midpoint """                
        rectangle_corners_for_x_y = rectangle_corners.reshape((4,2))        
        x_center_px, y_center_px = self.midpoint_equation(rectangle_corners_for_x_y[0,:], rectangle_corners_for_x_y[2,:])        
        return (x_center_px, y_center_px)

    def transform_aruco_midpoint_to_metric_system(self, aruco_midpoint): 
   
        x_m = (0.25738586)*(aruco_midpoint[0]) + 0.05862189

        x_px_times_y_px = aruco_midpoint[0]*aruco_midpoint[1]
        y_m = 0.29283879*aruco_midpoint[0] + 0.00050015*aruco_midpoint[1] + 0.00094536*x_px_times_y_px + 0.23096646

        x_px_times_z_px = aruco_midpoint[0]*aruco_midpoint[2]
        z_m = 0.16725805*aruco_midpoint[0] - 0.00069012*aruco_midpoint[2] + 0.00098029*x_px_times_z_px - 0.04520938         
        return (x_m, y_m, z_m)

    def cv2_to_imgmsg(self, image, encoding = "bgr8"):
        #print("cv2_to_imgmsg image shape is:" + str(image.shape))
        if encoding == "bgr8":
            self.curr_signs_image_msg.header = Header()
            self.curr_signs_image_msg.height = image.shape[0]
            self.curr_signs_image_msg.width = image.shape[1]
            self.curr_signs_image_msg.encoding = encoding
            self.curr_signs_image_msg.is_bigendian = 0
            self.curr_signs_image_msg.step = image.shape[1]*image.shape[2]

            data = np.reshape(image, (self.curr_signs_image_msg.height, self.curr_signs_image_msg.step) )
            data = np.reshape(image, (self.curr_signs_image_msg.height*self.curr_signs_image_msg.step) )
            data = list(data)
            self.curr_signs_image_msg.data = data
            return self.curr_signs_image_msg
        else:            
            raise Exception("Error while convering cv image to ros message") 
    
    def imgmsg_to_cv2(self, ros_image):
        if (ros_image.encoding == "bgr8" or ros_image.encoding == "rgb8") and ros_image.is_bigendian == 0:
            cv_img = np.array(list(ros_image.data), dtype= np.uint8)
            cv_img = np.reshape(cv_img, (ros_image.height, ros_image.step))
            cv_img = np.reshape(cv_img, (ros_image.height, ros_image.width, int(ros_image.step/ros_image.width) ) )
        else:
            cv_img = None
            raise Exception("Error while convering ros image message to a cv image")                  
        return cv_img
    
    def get_laser_index_from_angle(self, angle_in_deg):
        angle_in_deg_only_positive = angle_to_only_possitive_deg(angle_in_deg)        
        angle_index = round( (angle_in_deg_only_positive*len(self.scan.ranges))/360.0 )
        return int(angle_index)
    
    def get_mean_laser_value_at_fov(self, fov_center, fov_range):        
        if fov_range > 180.0:
            Exception("fov is too large")
        
        if fov_center - (fov_range/2.0) < 0.0:
            lower_boundary = fov_center - (fov_range/2.0)            
            values_at_fov_1 = self.scan.ranges[self.get_laser_index_from_angle(lower_boundary) : ]
            values_at_fov_2 = self.scan.ranges[: self.get_laser_index_from_angle( fov_center + (fov_range/2.0) ) + 1]
            values_at_fov = np.array(values_at_fov_1 + values_at_fov_2 )
        elif fov_center + (fov_range/2.0) > 360.0:
            values_at_fov_1 = self.scan.ranges[self.get_laser_index_from_angle(fov_center - (fov_range/2.0)) : ]
            values_at_fov_2 = self.scan.ranges[: self.get_laser_index_from_angle( (fov_center + (fov_range/2.0))%360.0 ) + 1]
            values_at_fov = np.array(values_at_fov_1 + values_at_fov_2 )
        else:
            values_at_fov = np.array( self.scan.ranges[ self.get_laser_index_from_angle(fov_center-(fov_range/2.0)) : self.get_laser_index_from_angle(fov_center+(fov_range/2.0)) + 1 ] )        

        return values_at_fov.mean()
    
    def euclidean_distance(self, tuple):
        return math.sqrt(tuple[0]**2 + tuple[1]**2 + tuple[2]**2)

    def calculate_beta(self, aruco_midpoint_angle):
        """angle_apperture = 0.15708 # 10 degrees at radius
        #aruco_midpoint_scan_index = self.get_laser_index_from_angle(self.aruco_midpoint_angle)
        #second_scan_index = self.get_laser_index_from_angle(self.aruco_midpoint_angle + angle_apperture)
        aruco_midpoint_scan_index = self.get_laser_index_from_angle(0.706858)
        second_scan_index = self.get_laser_index_from_angle(0.706858 + angle_apperture)
        a = aruco_midpoint_scan_index
        b = second_scan_index
        a = self.scan.ranges[a]
        b = self.scan.ranges[b]
        print ("a: ",a, "b: ",b)
        beta_complement = np.arcsin((b * np.sin(angle_apperture)) / (np.sqrt(a**2 + b**2 - 2*a*b*np.cos(angle_apperture))))
        beta = 90 - beta_complement"""
        
        beta = aruco_midpoint_angle + self.current_angle
        return beta

    def id2coordinate(self, id):
        aruco_coordinate = self.arucoCoordinates[str(id)]
        return aruco_coordinate
        
    def get_aruco_area_given_corners(self, unshaped_corners):
        corners = unshaped_corners.reshape((4,2))
        corners_for_det = np.concatenate((corners, corners[0,:].reshape((1,2))), axis = 0)
        det1 = np.linalg.det( corners_for_det[:2,:] )
        det2 = np.linalg.det( corners_for_det[1:3,:] )
        det3 = np.linalg.det( corners_for_det[2:4,:] )
        det4 = np.linalg.det( corners_for_det[3:5,:] )
        area = (det1 + det2 + det3 + det4)/2.0
        return area
    
    def filter_to_only_biggest_area_aruco(self, aruco_corners, aruco_ids):        
        aruco_areas = list(map(self.get_aruco_area_given_corners, aruco_corners))
        aruco_corners_areas_and_ids = list(zip(aruco_areas, aruco_corners, aruco_ids))
        aruco_corners_areas_and_ids.sort(reverse = True, key = lambda x:x[0])
        return (aruco_corners_areas_and_ids[0][1], aruco_corners_areas_and_ids[0][2])


    def min_possible(self, aruco_id, beta, aruco_angle):
        #aruco_midpoint_scan_index = self.get_laser_index_from_angle(aruco_angle)
        #r = self.scan.ranges[aruco_midpoint_scan_index]
        r = self.get_mean_laser_value_at_fov(aruco_angle, 2.0)
        aruco = self.id2coordinate(aruco_id)
        xAr, yAr = aruco
        a = (xAr-(np.cos(beta)*r)-self.arucoBoxDim/2.0 , yAr-(np.sin(beta)*r))
        b = (xAr+(np.sin(beta)*r) , yAr-(np.cos(beta)*r)-self.arucoBoxDim/2.0)
        c = (xAr+(np.cos(beta)*r)+self.arucoBoxDim/2.0 , yAr+(np.sin(beta)*r))
        d = (xAr-(np.sin(beta)*r) , yAr+(np.cos(beta)*r)+self.arucoBoxDim/2.0)
        possibilities = [a, b, c, d]
        dist = [euclidean_distance_point_to_point_2d(a,self.current_position_xy_2d),
                euclidean_distance_point_to_point_2d(b,self.current_position_xy_2d),
                euclidean_distance_point_to_point_2d(c,self.current_position_xy_2d),
                euclidean_distance_point_to_point_2d(d,self.current_position_xy_2d)]

        cuadrant = dist.index(min(dist))
        return possibilities[cuadrant]

    def main(self):

        #
        #print(self.image != None, self.scan != None, self.current_angle != None)
        
        if self.image != None and self.scan != None and self.current_angle != None:  
            print("entre")
            self.ocv_image = self.imgmsg_to_cv2(self.image)
            arucos_corners, arucos_ids = self.get_arucos_info_in_image(self.ocv_image)
            self.displayed_image_ocv = self.ocv_image.copy()
            
            if len(arucos_corners) > 0:                                        
                #aruco_corners, aruco_id = self.filter_to_only_biggest_area_aruco(arucos_corners, arucos_ids)
                #arucos_corner = [arucos_corners]
                for i in range(len(arucos_corners)):
                    aruco_corners = arucos_corners[i]
                    aruco_id = arucos_ids[i]
                    self.displayed_image_ocv = self.draw_arucos(self.displayed_image_ocv, arucos_corners)

                    pixel_midpoint = self.get_aruco_midpoint(aruco_corners)
                    aruco_angle = self.get_aruco_angle(pixel_midpoint)
                    aruco_angle = aruco_angle*(180.0/math.pi)
                    #aruco_angle = angle_to_only_possitive(aruco_angle)
                    #print("aplha value is: {v}".format(v = aruco_angle))

                    beta = self.calculate_beta (aruco_angle) 
                    x_calc, y_calc = self.min_possible(aruco_id[0], beta, aruco_angle)
                    #print("beta values is {v}".format(v = beta))
                    #print("x: ", x_calc, "y: ", y_calc)
                    print(aruco_id)
                
            

            self.curr_signs_image_msg = self.cv2_to_imgmsg(self.displayed_image_ocv, encoding = "bgr8")
            self.image_pub.publish(self.curr_signs_image_msg) 

            
   

if __name__ == "__main__":
    aruco_detector = ArucoDetector()
    
    rospy.spin()
    #aruco_detector.main()