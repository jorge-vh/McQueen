#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from tf2_geometry_msgs import PoseStamped

class Bug0Control(object):
    def __init__(self, goal_x, goal_y):
        # Inicializar el nodo de ROS
        rospy.init_node('bug0_control')

        # Publicar en el topic "/cmd_vel" para controlar el robot
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Suscribirse al topic "/scan" para recibir los datos del sensor Lidar
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        #rospy.Subscriber("/odom", Odometry, self.scan_callback)
        rospy.Subscriber('/pose_sim', PoseStamped, self.pose_callback)
        # Inicializar las variables de estado del algoritmo Bug0
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_reached = False
        self.distance_to_goal = 0.0
        self.distance_to_obstacle = float('inf')
        self.obstacle_detected = False
        self.robot_x = 0
        self.robot_y = 0
        #review the yaw cuz i dont know whats the correct parameter to take
        self.robot_yaw = 0

        # Inicializar la velocidad lineal y angular del robot
        self.linear_speed = 0.2
        self.angular_speed = 0.0

    def scan_callback(self, scan):
        # Obtener la distancia mínima al obstáculo del scan
        lidar_data = scan.ranges

        # Definir el rango de ángulos a considerar
        start_angle = 0
        end_angle = 180

        # Calcular los índices correspondientes a los ángulos en el rango deseado
        total_angles = len(lidar_data)
        start_index = int(total_angles * start_angle / 360)
        end_index = int(total_angles * end_angle / 360)

        # Seleccionar los rangos correspondientes a los ángulos en el rango deseado
        front_ranges = lidar_data[start_index:end_index]
        min_distance = min(front_ranges)
        if min_distance < 0.7:
            self.distance_to_obstacle = min_distance
            self.obstacle_detected = True
        else:
            self.distance_to_obstacle = float('inf')
            self.obstacle_detected = False

    def pose_callback(self,data):
        self.robot_x = data.pose.position.x
        self.robot_y = data.pose.position.y
        #is this the right one? Seems to be the one 
        self.robot_yaw = data.pose.orientation.z
        #print(self.robot_yaw)

    def run_bug0(self):
        # Mientras no se haya alcanzado la meta, ejecutar el algoritmo Bug0
        while not self.goal_reached and not rospy.is_shutdown():
            # Calcular la distancia al objetivo
            self.distance_to_goal = math.sqrt((self.goal_x - self.robot_x)**2 + (self.goal_y - self.robot_y)**2)
            print(self.distance_to_goal)
            # Si el robot está suficientemente cerca de la meta, detenerse
            if self.distance_to_goal < 0.1:
                self.goal_reached = True
                self.linear_speed = 0.0
                self.angular_speed = 0.0
            else:
                # Calcular el ángulo hacia la meta
                target_angle = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
                # Calcular la diferencia entre el ángulo hacia la meta y la orientación actual del robot
                angle_error = target_angle - self.robot_yaw
                #print(target_angle)
                #print(self.robot_yaw)
                #print("angle error: ",angle_error)
                # Si no se detectó un obstáculo, corregir la orientación del robot hacia la meta
                if not self.obstacle_detected:
                    # Si el ángulo de error es mayor que pi/2 o menor que -pi/2, girar hacia atrás
                    if angle_error > math.pi/2 or angle_error < -math.pi/2:
                        self.linear_speed = -0.2
                        self.angular_speed = 0.1
                        print("aqui")
                    else:
                        self.linear_speed = 0.1
                        self.angular_speed = 0.2*angle_error

                else:
                    # Si se detectó un obstáculo, girar hacia la izquierda hasta que no haya obstáculos
                   
                    self.linear_speed = 0.0
                    print("llegue aqui")
                    self.angular_speed = 0.2

            print(self.obstacle_detected)
            # Si hay un obstáculo cerca, seguir el contorno del obstáculo
            if self.obstacle_detected and self.distance_to_obstacle < 1.0:
                self.linear_speed = 0.0
                self.angular_speed = 0.2

            # Publicar la velocidad lineal y angular del robot
            twist = Twist()
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)

            # Esperar 0.1 segundos antes de la siguiente iteración
            rospy.sleep(0.1)

    def spin(self):
        # Ejecutar el algoritmo Bug0
        self.run_bug0()

if __name__ == '__main__':
# Coordenadas de la meta
    goal_x = 3.0
    goal_y = 0.0
    # Crear un objeto Bug0Control
    bug0 = Bug0Control(goal_x, goal_y)

    # Ejecutar el nodo
    try:
        bug0.spin()
    except rospy.ROSInterruptException:
        pass
