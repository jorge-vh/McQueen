#!/usr/bin/env python3
"""
@file
@brief This script provides the implementation of an ArUco detector node in ROS.
"""

import rospy
import cv2

class ArucoDetector():
    """
    @brief Class that represents the ArUco detector node.
    """

    def __init__(self, aruco_dict=cv2.aruco.DICT_4X4_50):
        """
        @brief Constructor of the ArucoDetector class.
        @param aruco_dict: ArUco dictionary used for marker detection.
        """
        self.vid = cv2.VideoCapture(0)
        rospy.init_node("aruco_detector")

    def main(self):
        """
        @brief Main function that runs the ArUco detector node.
        """
        while not rospy.is_shutdown():
            # Capture the video frame
            ret, frame = self.vid.read()
        
            # Display the resulting frame
            cv2.imshow('frame', frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
  
        self.vid.release()
        cv2.destroyAllWindows() 

if __name__ == "__main__":
    aruco_detector = ArucoDetector()
    aruco_detector.main()
