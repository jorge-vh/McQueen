    #!/usr/bin/env python
    import rospy
    import cv2
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image
    from std_msgs.msg import UInt16
    import numpy as np
    import time
    # Import the Kalman filter module
    from filterpy.kalman import KalmanFilter
    #sudo chmod 777 /dev/ttyACM0
    #rosrun rosserial_python serial_node.py /dev/ttyACM0
    # Constants for P control
    KP_angular = 0.0015


    # Constants for velocity limits
    MAX_ANGULAR_VELOCITY = 0.015  # Maximum angular velocity
    LINEAR_VELOCITY = 0.05  # Constant linear velocity


    # Constants for PID control
    Kp = 1.0  # Proportional gain
    Ki = 0.0 # Integral gain
    Kd = 0.2  # Derivative gain


    # Global variables
    current_marker = None
    bridge = CvBridge()
    cv_image = None
    target_aruco_id = 0  # Default target ArUco ID


    # Error variables for PID control
    prev_error = 0.0
    integral = 0.0


    # Create a Kalman filter for tracking the marker position
    kf = KalmanFilter(dim_x=4, dim_z=2)
    kf.F = np.array([[1, 0, 1, 0],
                    [0, 1, 0, 1],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    kf.H = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0]])
    kf.P *= 10  # Initial uncertainty covariance
    kf.R = np.diag([0.5, 0.5])  # Measurement noise covariance




    class Marker:
    def __init__(self):
        self.id = None
        self.x = None
        self.y = None
        self.marker_size = None
        
    # Callback function for image
    def image_callback(msg):
    global cv_image, current_marker


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
        largest_marker_index = np.argmax([cv2.contourArea(corner) for corner in corners])
        current_marker_id = ids[largest_marker_index][0]
        current_marker_corners = corners[largest_marker_index][0]


        # Get the center of the marker
        marker_center_x = (current_marker_corners[0][0] + current_marker_corners[2][0]) / 2
        marker_center_y = (current_marker_corners[0][1] + current_marker_corners[2][1]) / 2


        # Create a Marker object for the current marker
        current_marker = Marker()
        current_marker.id = current_marker_id


        # Apply Kalman filter to smooth the marker position
        measurement = np.array([[marker_center_x], [marker_center_y]])
        kf.predict()
        kf.update(measurement)
        current_marker.x = kf.x[0, 0]
        current_marker.y = kf.x[1, 0]


        # Calculate the marker size
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
    def control_loop(target_aruco_id):
    rospy.init_node('aruco_follower')


    # Subscribe to image topic
    rospy.Subscriber('/video_source/raw', Image, image_callback)


    # Publish velocity commands
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    servoJoint_pub = rospy.Publisher('servo_joint', UInt16, queue_size=10)
    servoGripper_pub = rospy.Publisher('servo_gripper', UInt16, queue_size=10)

    # Create a Twist message for velocity commands
    twist_cmd = Twist()

    AbrirGripper = UInt16()
    CerrarGripper = UInt16()
    AbrirGripper.data = 0
    CerrarGripper.data = 50
    
    ReposarJoint = UInt16()
    LevantarJoint = UInt16()
    ReposarJoint.data = 15
    LevantarJoint.data = 105
    
    # Set the control rate in Hz
    rate = rospy.Rate(10)
    integral = 0.0
    prev_error = 0.0
    while not rospy.is_shutdown():
        if current_marker is not None and current_marker.id == target_aruco_id:
            # Calculate the error between the current marker position and the center of the image
            image_height, image_width, _ = cv_image.shape
            image_center_x = image_width // 2
            image_center_y = image_height // 2
            marker_center_x = current_marker.x
            marker_center_y = current_marker.y
            if marker_center_x is not None:
                error_angular = image_center_x - marker_center_x
            else:
                # Handle the case when the marker is not detected
                error_angular = 0  # Assign a default value or do something appropriate for your application


            # PID control
            # Calculate the proportional term
            proportional = Kp * error_angular
            # Calculate the integral term
            integral += Ki * error_angular
            # Calculate the derivative term
            derivative = Kd * (error_angular - prev_error)
            # Calculate the angular velocity using PID control
            angular_velocity = proportional + integral + derivative
            # Limit angular velocity
            angular_velocity = min(MAX_ANGULAR_VELOCITY, max(-MAX_ANGULAR_VELOCITY, angular_velocity))
            # Store the current error for the next iteration
            prev_error = error_angular
            
            # Calculate the area of the marker
            if current_marker is not None and current_marker.marker_size is not None:
                marker_area = current_marker.marker_size * current_marker.marker_size
            else:
                # Handle the case when no marker is detected or the marker size is unknown
                marker_area = 0  # Assign a default value or do something appropriate for your application


            # If the marker area covers more than 10% of the image, stop the robot
            if marker_area > 0.07 * image_width * image_height and (current_marker.id == 0 or current_marker.id == 2 or current_marker.id == 1):
                twist_cmd.linear.x = 0.0
                twist_cmd.angular.z = 0.0
                cmd_vel_pub.publish(twist_cmd)
                servoGripper_pub.publish(CerrarGripper)
                time.sleep(1)
                servoJoint_pub.publish(LevantarJoint)
                time.sleep(1)
                return  # Exit the function to search for another ArUco
            #RECOLECCION
            if marker_area > 0.18 * image_width * image_height and (current_marker.id == 10 or current_marker.id == 11 or current_marker.id == 12):
                twist_cmd.linear.x = 0.0
                twist_cmd.angular.z = 0.0
                cmd_vel_pub.publish(twist_cmd)
                servoGripper_pub.publish(AbrirGripper)
                time.sleep(1)
                servoJoint_pub.publish(ReposarJoint)
                time.sleep(1)
                return  # Exit the function to search for another ArUco


            else:
                # If the marker is not centered, rotate with the calculated angular velocity
                if abs(error_angular) > 30:
                    #twist_cmd.linear.x = 0.0
                    twist_cmd.angular.z = angular_velocity
                else:
                    # If the marker is centered, move forward with a constant linear velocity
                    twist_cmd.linear.x = LINEAR_VELOCITY
                    twist_cmd.angular.z = angular_velocity


            # Publish the velocity command
            cmd_vel_pub.publish(twist_cmd)


            # Print the ID of the current marker
            print("Current marker ID:", current_marker.id, "marker_area: ", marker_area, "OBJETIVO: ", 0.05 * image_width * image_height )


        else:
            # If no marker is detected or the detected marker has a different ID, stop the robot
            twist_cmd.linear.x = 0.0
            twist_cmd.angular.z = 0.05
            cmd_vel_pub.publish(twist_cmd)


        rate.sleep()


    if __name__ == '__main__':
    # Set the IDs of the target ArUcos
    target_aruco_ids = [0, 10, 2, 12]


    rospy.init_node('aruco_follower')


    for target_aruco_id in target_aruco_ids:
        print("Following ArUco with ID:", target_aruco_id)


        control_loop(target_aruco_id)


