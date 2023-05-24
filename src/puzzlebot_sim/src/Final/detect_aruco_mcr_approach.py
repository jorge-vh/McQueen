#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from nav_functions import *
import nav_functions

class ArucoDetector():
    """
    Clase para detectar y procesar marcadores ArUco en imágenes de la cámara.
    """
    def __init__(self, aruco_dict=cv2.aruco.DICT_4X4_50):
        """
        Inicializa la clase ArucoDetector.

        Args:
            aruco_dict (int): Diccionario de marcadores ArUco predefinidos.
        """
        rospy.init_node("aruco_detector")

        # Atributos de ArUco
        self.arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.arucoDetector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)
        aruco_cordinates_for_challenge_world = {
            "0": (2.5, 0.0),
            "1": (2.5, 5.5),
            "2": (7.5, 5.5),
            "3": (7.5, 0.0),
            "4": (7.5, -5.5),
            "5": (2.5, -5.5)
        }
        self.arucoCoordinates = aruco_cordinates_for_challenge_world

        # Atributos de ROS        
        self.image_pub = rospy.Publisher("/image_detecting", Image, queue_size=1)
        self.estimated_sensor_reading_pub = rospy.Publisher("/estimated_sensor_reading", Float64MultiArray, queue_size=1) 
        self.real_sensor_reading_pub = rospy.Publisher("/real_sensor_reading", Float64MultiArray, queue_size=1)
        self.estimated_visual_sensor_reading_msg = Float64MultiArray() 
        self.real_visual_sensor_reading_msg = Float64MultiArray()
        self.relation_matrix_between_sensor_and_state_pub = rospy.Publisher("/relation_matrix_between_sensor_and_state", Float64MultiArray, queue_size=1)        
        self.relation_matrix_between_sensor_and_state_msg = Float64MultiArray()
        self.odom_sub = rospy.Subscriber('/kalman_corrected_odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        # Atributos de imagen      
        self.curr_signs_image_msg = Image()        

        # Atributos de color
        self.green = (0, 255, 0)

        # Atributos de video
        self.displayed_image_ocv = np.zeros(5, dtype=np.uint8)
        self.image = None
        self.ocv_image = None

        self.scan = None

        self.current_position_xy_2d = None
        self.current_angle = None        

        self.rate = rospy.Rate(5.0)

    def image_callback(self, msg):
        """
        Callback para recibir imágenes de la cámara.

        Args:
            msg: Mensaje de imagen de la cámara.
        """
        self.image = msg

    def scan_callback(self, msg):
        """
        Callback para recibir datos de escaneo láser.

        Args:
            msg: Mensaje de escaneo láser.
        """
        self.scan = msg

    def odom_callback(self, data):     
        """
        Callback para recibir datos de odometría.

        Args:
            data: Datos de odometría.
        """
        self.bypass_odom = data   
        self.current_position_xy_2d = (data.pose.pose.position.x, data.pose.pose.position.y)                             
        _, _, yaw = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.current_angle = yaw

    def midpoint_equation(self, p1, p2):
        """
        Calcula el punto medio entre dos puntos.

        Args:
            p1 (tuple): Coordenadas del primer punto.
            p2 (tuple): Coordenadas del segundo punto.

        Returns:
            tuple: Coordenadas del punto medio.
        """
        return ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2)

    def get_aruco_midpoint(self, rectangle_corners):
        """
        Obtiene las coordenadas del punto medio del marcador ArUco.

        Args:
            rectangle_corners (numpy.array): Esquinas del marcador ArUco.

        Returns:
            tuple: Coordenadas del punto medio del marcador ArUco.
        """
        rectangle_corners_for_x_y = rectangle_corners.reshape((4, 2))        
        x_center_px, y_center_px = self.midpoint_equation(rectangle_corners_for_x_y[0, :], rectangle_corners_for_x_y[2, :])        
        return (x_center_px, y_center_px)

    def fill_sensor_data_multi_array_layout(self):
        """
        Rellena el diseño de matriz multidimensional para los datos del sensor.
        """
        layout = MultiArrayLayout()
        dimension_0 = MultiArrayDimension()
        dimension_0.label = "len"
        dimension_0.size = 2
        dimension_0.stride = 2

        layout.dim[0] = dimension_0       
        layout.data_offset = 0
        
        self.real_visual_sensor_reading_msg.layout = layout

    def fill_relation_matrix_between_sensor_and_state_multi_array_layout(self):
        """
        Rellena el diseño de matriz multidimensional para la matriz de relación entre el sensor y el estado.
        """
        layout = MultiArrayLayout()
        
        dimension_0 = MultiArrayDimension()
        dimension_0.label = "height"
        dimension_0.size = 2
        dimension_0.stride = 2 * 3

        dimension_1 = MultiArrayDimension()
        dimension_1.label = "width"
        dimension_1.size = 3
        dimension_1.stride = 3

        layout.dim[0] = dimension_0
        layout.dim[1] = dimension_1        
        layout.data_offset = 0
        
        self.relation_matrix_between_sensor_and_state_msg.layout = layout

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
    
    def euclidean_distance(self, tuple):
        return math.sqrt(tuple[0]**2 + tuple[1]**2)

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

    def get_laser_index_from_angle(self, angle_in_deg):
        angle_in_deg_only_positive = nav_functions.angle_to_only_possitive_deg(angle_in_deg)        
        angle_index = np.floor( (angle_in_deg_only_positive*len(self.scan.ranges))/360.0 )
        return int(angle_index)
    
    def get_alpha(self, aruco_midpoint, camera_fov = 1.3962634, image_width = 800.0):
        """
        input:
            aruco_midpoint : tuple with (x_center_px, y_center_px)
        output:
            aruco_angle: float with aruco angle in rads
        """
        angle = -(camera_fov/image_width)*aruco_midpoint[0] + camera_fov/2
        return angle

    def publish_sensor_data(self, aruco_cordinates, aruco_midpoint):
        if self.current_position_xy_2d != None and self.current_angle != None:
            delta_x = aruco_cordinates[0] - self.current_position_xy_2d[0]
            delta_y = aruco_cordinates[1] - self.current_position_xy_2d[1]
            print("*****CLOSEST ARUCO*****")
            print ("ArUco detected in:",delta_x,"from x")
            print ("ArUco detected in:",delta_y,"from y")
            p = self.euclidean_distance((delta_x, delta_y))
            alpha = np.arctan2(delta_y, delta_x) - self.current_angle
            
            relation_matrix_between_sensor_and_state = np.array(
                [[ -(delta_x/p), -(delta_y/p), 0.0],
                 [ (delta_y/(p**2)), -(delta_x/(p**2)), -1.0 ]]
            )
            flattened_relation_matrix_between_sensor_and_state = relation_matrix_between_sensor_and_state.reshape((6, ))
            
            self.relation_matrix_between_sensor_and_state_msg.data = flattened_relation_matrix_between_sensor_and_state.tolist()
            
            self.estimated_visual_sensor_reading_msg.data = [p, alpha]

            real_alpha = self.get_alpha(aruco_midpoint)
            real_alpha_in_deg = np.rad2deg(real_alpha) 
            
            index = self.get_laser_index_from_angle(real_alpha_in_deg)

            real_p = self.scan.ranges[index]
            self.real_visual_sensor_reading_msg.data = [real_p, real_alpha]
            #self.real_sensor_reading_pub.publish(self.real_visual_sensor_reading_msg)

            if not np.isinf(real_p):
                self.relation_matrix_between_sensor_and_state_pub.publish(self.relation_matrix_between_sensor_and_state_msg)
                self.estimated_sensor_reading_pub.publish(self.estimated_visual_sensor_reading_msg)
                self.real_sensor_reading_pub.publish(self.real_visual_sensor_reading_msg)

    def main(self):
        while not rospy.is_shutdown():            
            if self.image != None and self.scan != None and self.current_position_xy_2d != None:                
                self.ocv_image = self.imgmsg_to_cv2(self.image)
                arucos_corners, arucos_ids = self.get_arucos_info_in_image(self.ocv_image)
                self.displayed_image_ocv = self.ocv_image.copy()
                
                if len(arucos_corners) > 0:
                    aruco_corners, aruco_id = self.filter_to_only_biggest_area_aruco(arucos_corners, arucos_ids)
                    arucos_corners = [aruco_corners]
                    self.displayed_image_ocv = self.draw_arucos(self.displayed_image_ocv, arucos_corners)
                    self.displayed_image_ocv = cv2.resize(self.displayed_image_ocv, (100,100), interpolation = cv2.INTER_AREA)

                    print("aruco id is: ", aruco_id[0])
                    aruco_cordinates = self.id2coordinate(aruco_id[0])
                    aruco_midpoint_px = self.get_aruco_midpoint(aruco_corners)
                    self.publish_sensor_data(aruco_cordinates, aruco_midpoint_px)                        

                self.curr_signs_image_msg = self.cv2_to_imgmsg(self.displayed_image_ocv, encoding = "bgr8")
                self.image_pub.publish(self.curr_signs_image_msg)
                                
            self.rate.sleep()
   

if __name__ == "__main__":
    aruco_detector = ArucoDetector()
    aruco_detector.main()