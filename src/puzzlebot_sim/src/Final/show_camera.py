#!/usr/bin/env python3

import rospy
import cv2


class ArucoDetector():
    def __init__(self, aruco_dict = cv2.aruco.DICT_4X4_50):
        self.vid = cv2.VideoCapture(0)
        rospy.init_node("aruco_detector")



    def main(self):
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