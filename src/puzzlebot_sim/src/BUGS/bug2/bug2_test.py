#!/usr/bin/env python

# Librerias
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

class bug2(object):
    def __init__(self, gX, gY):

        # Posicion del robot
        self.pos = Odometry()
        self.x = 0
        self.y = 0
        self.angle = 0

        # Velocidad del robot
        self.vw = Twist()

        #Obstaculos
        self.scan = LaserScan()
        self.front = float('inf')
        self.right = float('inf')

        # Control
        self.kv = 0.5
        self.kw = 0.1

        # Error
        self.ed = 0
        self.et = 0
        self.last_ed = 0
        self.last_et = 0

        # Meta
        self.xt = gX
        self.yt = gY
        self.distance_to_goal = 0
        self.distance_to_obstacle = 0.3
        self.goal_reached = False
        self.obstacle_detected = False

        # Linea-m
        self.m = (self.yt-self.pos.pose.pose.position.y) / (self.xt-self.pos.pose.pose.position.x) # m = (y2-y1) / (x2-x1)
        #self.m = math.tan(self.angle) # tan(theta)
        self.b = (self.pos.pose.pose.position.y) - (self.m*self.pos.pose.pose.position.x) # b = y-mx
        self.distance_to_line = 0

        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Suscribers
        rospy.Subscriber("/scan", LaserScan, self.scanCallback)
        rospy.Subscriber("/odom", Odometry, self.getPuzzlePos)

    # Callback de valores del Lidar
    def scanCallback(self, scan):
        self.scan = scan
        fl = min(scan.ranges[0:50])
        fr = min(scan.ranges[len(scan.ranges)-50:len(scan.ranges)-1])
        self.front = min(fl,fr)
        self.right = min(scan.ranges[792:920])
        print("Front: ", self.front)
        print("Right: ", self.right)

    # Callback de valores de odometria para calcular la posicion
    def getPuzzlePos(self, odom):
        self.pos = odom

    # Obtener el angulo correcto del robot
    def getRotation(self):
        orientation = self.pos.pose.pose.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return yaw
    
    #Funcion para calcular el error de posicion
    def error(self):
        self.last_et = self.et
        if(self.angle >= 0):
            self.et=math.atan2(self.yt,self.xt)-self.angle
        elif(self.angle < 0) :
            self.et=-math.atan2(self.yt,self.xt)-self.angle
        self.last_ed = self.ed
        self.ed = math.hypot(self.xt-self.x, self.yt-self.y)

    # Convertir angulo en un indice del arreglo de los rangos del Lidar
    def index(self, angle):
        ini_angle = self.scan.angle_min # angulo de inicio [rad]
        end_angle = self.scan.angle_max # angulo final [rad]
        n = len(self.scan.ranges)-1 # Ultimo indice del arreglo
        # Si el angulo se pasa del maximo, usar el ultimo indice del arreglo
        if angle > end_angle:
            return n
        # Si el angulo es menor al minimo, usar el primer dato del arreglo (i=0)
        elif angle < ini_angle:
            return 0
        # n grados del angulo maximo al minimo
        degrees = np.linspace(end_angle, ini_angle, n)
        # Encontrar el indice con menor diferencia con el angulo deseado
        i = (np.abs(degrees-angle)).argmin()
        return i

    # Obtener el angulo de error entre la pared y el robot
    def wallDirection(self):
        # Definir el angulo en radianes
        a_ini = np.pi/2 - np.pi/6
        a_end = np.pi/2 + np.pi/6
        theta = abs(a_end) - abs(a_ini)
        # Encontrar los indices de dichos angulos
        i_ini = self.index(a_ini)
        i_fin = self.index(a_end)
        # Obtener los valores de distancia
        a = self.scan.ranges[i_ini]
        b = self.scan.ranges[i_fin]
        # c = sqrt(a^2+b^2-2ab*cos(theta))
        c = np.sqrt(math.pow(a,2) + math.pow(b,2) - (2 * (a*b) * np.cos(abs(theta))))
        # angle = sin-1[(a-b*cos(theta))/c]
        angle = np.arcsin((a - b * np.cos(theta)) / c) 
        return angle

    # Seguir contorno del obstaculo
    def followObs(self):
        v = 0.15# Velocidad lineal al seguir la pared
        # Angulo de error con la pared
        angle_wall = self.wallDirection() 
        print("Angle wall:",angle_wall)
        # Constantes para aplicar control
        k_d = 0.1
        k_t = 0.1
        # Error entre la distancia maxima al obstaculo y la distancia medida por el lidar
        rED = self.distance_to_obstacle - self.right
        # Aplicar control para corregir el angulo entre el robot y la pared
        vw_angular = (angle_wall*k_t) + (rED*k_d)
        # Velocidad maxima angular en z
        max_vw_angular = 0.3
        # Si hay una pared enfrente, girar y quedar en paralelo a la pared
        if self.front <= self.distance_to_obstacle:  
            vw_angular = max_vw_angular
            print("Girando hacia la pared")
        # Esquinas
        elif np.isnan(angle_wall):
            vw_angular = -v/(self.distance_to_obstacle)
            print("Esquina")
        # Asegurarse de que el valor de vel angular no se pase de los limites
        z = np.clip(vw_angular, -max_vw_angular, max_vw_angular)
        # Publicar velocidades
        self.vw.linear.x = v
        self.vw.angular.z = z
        self.cmd_vel_pub.publish(self.vw)

    # Definir linea-m del robot a la meta
    def lineM(self, x, y, m, b):
        # d=[abs(A(x)+B(y)+C)]/[sqrt(A^2)+B^2] 
        # Obtener coeficientes A, B, C
        A = -m
        B = 1
        C = -B*b
        # Calcular distancia   
        d = abs(A*x + B*y + C) / math.sqrt(pow(A,2)+pow(B,2))
        return d
   
    # Funcion principal 
    def main(self):
        # Calcular la posicion actual del robot
        self.x = self.pos.pose.pose.position.x
        self.y = self.pos.pose.pose.position.y
        self.angle = self.getRotation()
        #print("Pos: ", round(self.x,2), round(self.y,2), round(self.angle,2))
        # Calcular la distancia a la meta
        self.error()
        #print("Error: ", round(self.ed,2), round(self.et,2))
        # Calcular la distancia a la linea-m
        self.distance_to_line = self.lineM(self.x,self.y,self.m,self.b)
        #Ir hacia la meta
        if self.goal_reached == False and self.obstacle_detected == False:
            print("x: ", self.x)
            print("y: ", self.y)
            # Llego a la meta
            if self.ed < 0.1:
                self.goal_reached == True
                # Detenerse
                self.vw.linear.x=0
                self.vw.angular.z=0
                self.cmd_vel_pub.publish(self.vw)
                print("Llegue a la meta")
                rospy.spin()
            
            # Avanzar
            else: 
                # Girar
                if self.et > 0.03:
                    self.vw.linear.x = 0
                    if self.et > 0:
                        self.vw.angular.z = self.et/2
                    elif self.et < 0:
                        self.vw.angular.z = -self.et/2
                    print("Girando")
                # Avanzar derecho hacia la meta                  
                else:
                    self.vw.linear.x = 0.1
                    self.vw.angular.z = 0
                #self.vw.linear.x = 0.15
                #self.vw.angular.z = self.et  
                print("Avanzando")
                # Publicar las velocidades
                self.cmd_vel_pub.publish(self.vw)
                
                # Cambiar a seguir el obstaculo
                if self.front < self.distance_to_obstacle:
                    self.obstacle_detected = True
                

        # Encontro un obstaculo
        elif self.goal_reached == False and self.obstacle_detected == True: 
            print("Siguiendo pared")
            self.followObs()
            print("Dist a la linea:", self.distance_to_line)
            # Dejar la pared si encuentra la linea-m
            if (self.distance_to_line) <= 0.01:
                self.obstacle_detected = False
                print("Continuar llendo hacia la meta")

        else:
            print("Algo salio mal")
            print("No entre en ningun caso")

# Main
if __name__ == '__main__':
    rospy.init_node('bug2')
    # Posicion de la meta
    gX = 2.0
    gY = 0
    b = bug2(gX, gY)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            b.main()
            rate.sleep()
        except rospy.ROSInterruptException:
            pass

