#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf2_geometry_msgs import PoseStamped

class Bug0Control(object):
    def __init__(self, goal_x, goal_y):
        # Inicializar el nodo de ROS
        rospy.init_node('bug0_control')

        # Publicar en el topic "/cmd_vel" para controlar el robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/pose_sim', PoseStamped, self.pose_callback)
        # Suscribirse al topic "/scan" para recibir los datos del sensor Lidar
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Inicializar las variables de estado del algoritmo Bug0
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_reached = False
        self.distance_to_goal = 0.0
        self.distance_to_obstacle = float('inf')
        self.obstacle_detected = False
        self.robot_x = 0
        self.robot_y = 0

        # Inicializar la velocidad lineal y angular del robot
        self.linear_speed = 0.2
        self.angular_speed = 0.0

    def pose_callback(self,data):
        self.robot_x = data.pose.position.x
        self.robot_y  = data.pose.position.x

    def scan_callback(self, scan):
        # Obtener la distancia mínima al obstáculo del scan
        #min_distance = min(scan.ranges)
        front_ranges = scan.ranges[0:200]
        min_distance = min(front_ranges)
        if min_distance < 0.7:
            self.distance_to_obstacle = min_distance
            self.obstacle_detected = True
        else:
            self.distance_to_obstacle = float('inf')
            self.obstacle_detected = False

    def run_bug0(self):
        # Mientras no se haya alcanzado la meta, ejecutar el algoritmo Bug0
        while not self.goal_reached and not rospy.is_shutdown():
            # Calcular la distancia al objetivo
            self.distance_to_goal = math.sqrt((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)

            # Si se detectó un obstáculo, girar hacia la izquierda hasta que no haya obstáculos
            print(self.obstacle_detected)
            if self.obstacle_detected:
                self.linear_speed = 0.0
                self.angular_speed = 0.4
            else:
                self.linear_speed = 0.2
                self.angular_speed = 0.0

            # Si el robot está suficientemente cerca de la meta, detenerse
            if self.distance_to_goal < 0.1:
                self.goal_reached = True
                self.linear_speed = 0.0
                self.angular_speed = 0.0

            # Publicar la velocidad en el topic "/cmd_vel"
            cmd_vel = Twist()
            cmd_vel.linear.x = self.linear_speed
            cmd_vel.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(cmd_vel)

            # Esperar un tiempo antes de continuar la ejecución del bucle
            rospy.sleep(0.1)

if __name__ == '__main__':
    # Crear una instancia del controlador Bug0 con las coordenadas de la meta
    bug0 = Bug0Control(1.0, 1.0)

    # Ejecutar el algoritmo Bug0
    bug0.run_bug0()
