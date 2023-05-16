#!/usr/bin/env python3
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3,Quaternion,PoseStamped
from std_msgs.msg import String
import math

class ObstacleDetection:
    
    def __init__(self):
        # Configurar el nodo de ROS
        rospy.init_node('obstacle_detection')
        
        # Suscribirse al tópico de LaserScan
        rospy.Subscriber('/scan', LaserScan, self.callback)
        
        # Publicar en el tópico de String
        self.pub = rospy.Publisher('/obstacle_location', String, queue_size=10)
        rospy.Subscriber('/pose_sim',PoseStamped,self.pose_callback)
        self.quats = Quaternion()
        self.position = Vector3()
        self.vecObs = []
        self.vecU = []
        self.orientation = 0

    def getAngle(self, vector_1, vector_2): #Returns the angle between two vectors
        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)
        self.eThetaVec = np.arccos(dot_product)

    def pose_callback(self, data):
        # Actualizar la posición actual del robot
        self.position = data.pose.position
        self.quats = data.pose.orientation
        self.euler_from_quaternion()

    def euler_from_quaternion(self):
        t3 = +2.0 * (self.quats.w * self.quats.z + self.quats.x * self.quats.y)
        t4 = +1.0 - 2.0 * (self.quats.y * self.quats.y + self.quats.z * self.quats.z)
        self.orientation = math.atan2(t3, t4)

    def callback(self, data):
        # Obtener los valores del rango de medición del LiDAR
        ranges = data.ranges
        npRanges = np.empty(len(ranges))
        npRanges[:] = ranges
        min_index = np.argmin(npRanges)

        self.vecObs = [math.cos(min_index*data.angle_increment),math.sin(min_index*data.angle_increment)]
        self.vecU = [-self.vecObs[1],self.vecObs[0]]
        self.vecOrientation = [math.cos(self.orientation),math.sin(self.orientation)]
        #print(min_index)
        #print(min_index*data.angle_increment)
        #print(self.orientation)
        print("Vector del laser")
        print(self.vecObs)
        print("Vector perpendicular la laser")
        print(self.vecU)
        print("vector de orientacion del robot")
        print(self.vecOrientation)
        # Calcular el valor mínimo de la medición
        min_range = min(ranges)
        # Si el valor mínimo es menor que un umbral, significa que hay un obstáculo
        if min_range < 1.0:
            # Obtener el índice del valor mínimo
            min_index = ranges.index(min_range)
            
            # Publicar en el tópico de String la dirección del obstáculo
            if min_index < len(ranges) / 2:
                self.pub.publish('Obstacle detected on the left')
            else:
                self.pub.publish('Obstacle detected on the right')
        else:
            # Si no hay obstáculos, publicar en el tópico de String "sin muros en la costa"
            self.pub.publish('Sin muros en la costa')

if __name__ == '__main__':
    try:
        detector = ObstacleDetection()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
