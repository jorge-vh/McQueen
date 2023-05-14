#!/usr/bin/env python3

import rospy
from math import atan2, pi, cos, sin, sqrt
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class Bug0Algorithm:
    def __init__(self):
        rospy.init_node('bug0', anonymous=True)
        # Posición actual del robot
        self.position = Vector3()

        # Posición de la meta
        self.goal_position = Vector3(2.0, 0.0, 0.0)

        # Posición del obstáculo más cercano
        self.obstacle_position = Vector3()

        # Bandera que indica si se ha alcanzado la meta
        self.goal_reached = False

        # Bandera que indica si se ha detectado un obstáculo
        self.obstacle_detected = False

        # Bandera que indica si se está siguiendo la pared
        self.following_wall = False

        # Dirección hacia la meta
        self.goal_direction = 0.0

        # Dirección hacia el obstáculo
        self.obstacle_direction = 0.0

        # Distancia al obstáculo más cercano
        self.obstacle_distance = 0.0
        self.obstacle_threshold = 0.5
        # Distancia al objetivo
        self.goal_distance = 0.0

        # Ángulo de corrección
        self.correction_angle = 0.0

        # Suscriptor a la odometría
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Suscriptor al LIDAR
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # Publicador del mensaje de control
        self.control_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Tasa de publicación del mensaje de control
        self.rate = rospy.Rate(10)

    def odometry_callback(self, data):
        # Actualización de la posición actual del robot
        self.position = data.pose.pose.position

    def lidar_callback(self, data):
        # Cálculo de la posición del obstáculo más cercano
        min_distance = data.range_max
        min_angle = 0.0
        for i, distance in enumerate(data.ranges):
            angle = data.angle_min + i * data.angle_increment
            if distance < min_distance:
                min_distance = distance
                min_angle = angle
        if min_distance < 0.2:
            self.obstacle_detected = True
            print("encontre un objeto")
            self.obstacle_distance = min_distance
            self.obstacle_direction = self.calculate_obstacle_direction()
            self.obstacle_position.x = self.position.x + min_distance * cos(self.position.z + min_angle)
            self.obstacle_position.y = self.position.y + min_distance * sin(self.position.z + min_angle)
        else:
            self.obstacle_detected = False

    def calculate_distance(self):
        # Cálculo de la distancia a la meta
        dx = self.goal_position.x - self.position.x
        dy = self.goal_position.y - self.position.y
        return sqrt(dx**2 + dy**2)

    def calculate_direction(self):
        # Cálculo de la dirección hacia la meta
        dx = self.goal_position.x - self.position.x
        dy = self.goal_position.y - self.position.y
        return atan2(dy, dx)

    def calculate_obstacle_direction(self):
        # Cálculo de la dirección hacia el obstáculo
        dx = self.obstacle_position.x - self.position.x
        dy = self.obstacle_position.y - self.position.y
        return atan2(dy, dx)

    def calculate_correction_angle(self):
        # Cálculo del ángulo de corrección
        if abs(self.goal_direction - self.position.z) > pi:
            if self.goal_direction < self.position.z:
                self.goal_direction += 2*pi
            else:
                self.position.z += 2*pi
        return self.goal_direction - self.position.z

    def move_to_goal(self):
        # Movimiento hacia la meta
        while not rospy.is_shutdown() and not self.goal_reached:
            if not self.obstacle_detected:
                # Si no hay obstáculos, avanzar hacia la meta
                self.goal_distance = self.calculate_distance()
                if self.goal_distance < 0.1:
                    self.goal_reached = True
                    self.stop_moving()
                else:
                    self.goal_direction = self.calculate_direction()
                    self.rotate_to_goal()
                    self.move_forward()
            else:
                # Si hay obstáculos, seguir la pared
                self.follow_wall()

            self.rate.sleep()

    def follow_wall(self):
        # Seguir la pared
        self.following_wall = True
        while not rospy.is_shutdown() and self.following_wall:
            if self.obstacle_detected:
                # Si el obstáculo sigue siendo detectado, seguir la pared
                self.obstacle_direction = self.calculate_obstacle_direction()
                if abs(self.obstacle_direction - self.position.z) > pi:
                    if self.obstacle_direction < self.position.z:
                        self.obstacle_direction += 2*pi
                    else:
                        self.position.z += 2*pi
                self.correction_angle = self.obstacle_direction - self.position.z
                self.move_forward()
            else:
                # Si el obstáculo no está siendo detectado, volver a la ruta original
                self.following_wall = False

    def rotate_to_goal(self):
        # Rotación hacia la dirección de la meta
        self.correction_angle = self.calculate_correction_angle()
        if abs(self.correction_angle) > 0.1:
            angular_speed = min(0.5, 0.5*abs(self.correction_angle))
            if self.correction_angle > 0:
                twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, -angular_speed))
            else:
                twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, angular_speed))
            self.control_pub.publish(twist)
            self.rate.sleep()

    def move_forward(self):
        # Movimiento hacia adelante
        if self.goal_reached or self.following_wall:
            # Si se ha alcanzado la meta o se está siguiendo la pared, detenerse
            self.stop_moving()
        else:
            # Si no, avanzar hacia adelante
            linear_speed = min(0.5, 0.3*self.goal_distance)
            twist = Twist(linear=Vector3(linear_speed, 0, 0), angular=Vector3(0, 0, 0))
            self.control_pub.publish(twist)
            self.rate.sleep()

    def stop_moving(self):
        # Detener el movimiento
        twist = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
        self.control_pub.publish(twist)

    def lidar_callback(self, msg):
        # Callback de LIDAR
        self.obstacle_detected = False
        for r in msg.ranges:
            if r < self.obstacle_threshold:
                self.obstacle_detected = True
                self.obstacle_distance = r
                break
        if self.obstacle_detected:
            self.obstacle_direction = self.calculate_obstacle_direction()
            if abs(self.obstacle_direction - self.position.z) > pi:
                if self.obstacle_direction < self.position.z:
                    self.obstacle_direction += 2*pi
                else:
                    self.position.z += 2*pi
            self.correction_angle = self.obstacle_direction - self.position.z

    def odom_callback(self, msg):
        # Callback de odometría
        self.position.x = msg.pose.pose.position.x
        self.position.y = msg.pose.pose.position.y
        self.position.z = msg.pose.pose.orientation.z

    def run(self):
        # Loop principal
        self.move_to_goal()
        

if __name__ == '__main__':
    try:
        node = Bug0Algorithm()
        node.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
          
