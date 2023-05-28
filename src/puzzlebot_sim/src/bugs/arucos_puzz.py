#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np

# Constants for P control
KP_angular = 0.003

# Constants for velocity limits
MAX_ANGULAR_VELOCITY = 0.02  # Maximum angular velocity
LINEAR_VELOCITY = 0.05  # Constant linear velocity

# Global variables
current_marker = None
bridge = CvBridge()
cv_image = None

class Marker:
    def _init_(self):
        self.id = None
        self.x = None
        self.y = None
        self.marker_size = None

# Callback function for image
def image_callback(msg):
    global cv_image

    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except:
        return

    # Get the image size
    image_height, image_width, _ = cv_image.shape

    # Convert the image to grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Define the dictionary of ArUco markers and parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters_create()

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # Find the marker with the largest size
        largest_marker_index = np.argmax(ids)
        current_marker_id = ids[largest_marker_index][0]
        current_marker_corners = corners[largest_marker_index][0]

        # Get the center of the marker
        marker_center_x = (current_marker_corners[0][0] + current_marker_corners[2][0]) / 2
        marker_center_y = (current_marker_corners[0][1] + current_marker_corners[2][1]) / 2

        # Create a Marker object for the current marker
        global current_marker
        current_marker = Marker()
        current_marker.id = current_marker_id
        current_marker.x = marker_center_x
        current_marker.y = marker_center_y
        current_marker.marker_size = np.sqrt(
            (current_marker_corners[0][0] - current_marker_corners[2][0]) ** 2
            + (current_marker_corners[0][1] - current_marker_corners[2][1]) ** 2
        )

        # Draw a rectangle around the marker
        cv2.aruco.drawDetectedMarkers(cv_image, corners)

    else:
        current_marker = None

    # Resize the image window to 640x480
    #cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
    #cv2.resizeWindow("Image", 640, 480)
    #cv2.imshow("Image", cv_image)
    #cv2.waitKey(1)

# Main control loop
def control_loop():
    rospy.init_node('aruco_follower')

    # Subscribe to image topic
    rospy.Subscriber('/video_source/raw', Image, image_callback)

    # Publish velocity commands
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a Twist message for velocity commands
    twist_cmd = Twist()

    # Set the control rate in Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if current_marker is not None:
            # Calculate the error between the current marker position and the center of the image
            image_height, image_width, _ = cv_image.shape
            image_center_x = image_width // 2
            image_center_y = image_height // 2
            marker_center_x = current_marker.x
            marker_center_y = current_marker.y
            error_angular = image_center_x - marker_center_x

            # Calculate the angular velocity using P control
            angular_velocity = -KP_angular * error_angular

            # Limit angular velocity
            angular_velocity = min(MAX_ANGULAR_VELOCITY, max(-MAX_ANGULAR_VELOCITY, angular_velocity))

            # Calculate the area of the marker
            marker_area = current_marker.marker_size * current_marker.marker_size

            # If the marker area covers more than 10% of the image, stop the robot
            if marker_area > 0.1 * image_width * image_height:
                twist_cmd.linear.x = 0.0
                twist_cmd.angular.z = 0.0
            else:
                # If the marker is not centered, rotate with a proportional angular velocity
                if abs(error_angular) > 20:
                    #twist_cmd.linear.x = 0.0
                    twist_cmd.angular.z = -angular_velocity
                else:
                    # If the marker is centered, move forward with a constant linear velocity
                    twist_cmd.linear.x = LINEAR_VELOCITY
                    twist_cmd.angular.z = -angular_velocity

            # Publish the velocity command
            cmd_vel_pub.publish(twist_cmd)

        else:
            # If no marker is detected, stop the robot
            twist_cmd.linear.x = 0.0
            twist_cmd.angular.z = 0.3
            cmd_vel_pub.publish(twist_cmd)

        rate.sleep()

if __name__ == '_main_':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass