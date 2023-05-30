#!/usr/bin/env python
# Codigo del equipo 3 para la deteccion y seguimiento de arucos
import cv2
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import atan2, sqrt, pow
import time

class Puzzlebot:
    def __init__(self):
        # Datos del puzzlebot
        self.r = 0.2
        self.freq = 25
        self.l = 10
        self.bridge = CvBridge()
        # Posicion y velocidad inicial
        self.x = 0
        self.y = 0
        self.theta = 0
        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0
        # Variables para el control PID
        self.last_ed = 0
        self.last_ey = 0
        self.last_et = 0
        self.total_ed = 0
        self.total_ey = 0
        self.total_et = 0
        self.kpa = 0.01
        self.kia = 0.005
        self.kda = 0.01
        self.kpl = 0.05
        self.kdl = 0.05
        self.ex = 0
        self.ey = 0
        self.ed = 0
        self.et = 0
        self.dt = 0.01
        self.remove = False
        # Publishers y subscribers
        rospy.Subscriber("/video_source/raw", Image, self.IBVS)
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def control(self):
        self.et = atan2(self.ey, self.ex)
        self.total_et += self.et
        d_et = (self.et - self.last_et) * self.dt
        i_et = self.total_et / self.dt
        self.vel.angular.z = (self.kpa * self.et + self.kda * d_et) * -1  # + self.ki*i_et
        self.vel.angular.z = max(self.vel.angular.z, 0.3)
        self.last_et = self.et

        self.ed = sqrt(pow(self.ex, 2) + pow(self.ey, 2))
        self.total_ed += self.ed
        d_ed = (self.ed - self.last_ed) * self.dt
        i_ed = self.total_ed / self.dt
        self.vel.linear.x = self.kpl * self.ed + self.kdl * d_ed  # + self.ki*i_ed
        self.vel.linear.x = max(self.vel.linear.x, 0.3)
        self.last_ed = self.ed

        if self.ex < 0.01 and self.ex > -0.01:
            self.vel.angular.z = 0
        if self.ex < -0.01:
            self.vel.angular.z *= -1
        self.vel_pub.publish(self.vel)
        print('Velocidad lineal en x: {}'.format(self.vel.linear.x))
        print('Velocidad angular en z: {}'.format(self.vel.angular.z))

    def IBVS(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        k = np.load("/home/puzzlebot/Documents/catkin/McQueen/src/puzzlebot_sim/src/BUGS/calibration_matrix.npy")
        d = np.load("/home/puzzlebot/Documents/catkin/McQueen/src/puzzlebot_sim/src/BUGS/distortion_coefficients.npy")
        arucoDict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        arucoParams = aruco.DetectorParameters_create()
        (corners, ids, rejected) = aruco.detectMarkers(img, arucoDict, parameters=arucoParams)
        if len(corners) <= 0:
            self.vel.angular.z = 0.2
            self.vel.linear.x = 0.3
            self.vel_pub.publish(self.vel)
        if len(corners) > 0:
            max_area = 0
            max_area_index = 0
            for i in range(len(corners)):
                area = cv2.contourArea(corners[i][0])
                if area > max_area:
                    max_area = area
                    max_area_index = i
            print("Aruco mas grande: ",max_area_index)
            rvec, tvec = aruco.estimatePoseSingleMarkers(corners[max_area_index], 0.02, k, d)
            self.ex = (tvec[0][0][0]) - 0.02
            self.ey = (tvec[0][0][2]) - 0.02
            if self.ex < 0.05 and self.ex > -0.05 and self.ey < 0.014:
                print("Llego")
                self.vel.linear.x = 1
                self.vel_pub.publish(self.vel)
                time.sleep(0.2)
            self.control()
            img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
        cv2.imshow('Aruco', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('puzzlebot_aruco')
    puzzlebot = Puzzlebot()
    rospy.spin()
