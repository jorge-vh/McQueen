#!/usr/bin/env python3
# Library that contains the basic files for ros to work
import rospy
from math import sin,cos,tan,atan2,pi
import math
import numpy
from sensor_msgs.msg import LaserScan
# Library that contains standard ROS msgs
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D,Twist

class control:
    #Se inicializan todas las variables para odometría y publicar velocidad del robot
    def __init__(self):
        #Variables para odometría
        self.wr=0
        self.wl=0
        self.r =0.05
        self.l =0.19
        self.loc=Pose2D()
        self.loc.x=0
        self.loc.y=0
        self.loc.theta=0
        self.dt=0
        #Variables para movimiento en X y Y
        self.goalY=0
        self.goalX=4
        self.etheta=0
        self.ed=0
        #Variables para movimiento del puzzlebot
        self.transform=Twist()
        self.transform.linear.x=0
        self.transform.linear.y=0
        self.transform.linear.z=0
        self.transform.angular.x=0
        self.transform.angular.y=0
        self.transform.angular.z=0
        #Constantes para control P
        self.kv=0.1
        self.kw=0.1
        #lidar
        self.obstacle_detected = False
        self.distance_to_obstacle = 10000

    #Función para calcular posición del puzzlebot
    def calculatePosition(self):
        #cálculo de theta
        self.loc.theta=self.loc.theta+self.r*((self.wr-self.wl)/self.l)*self.dt
        #condicionales para establecer un rango de pi a -pi
        if self.loc.theta >= numpy.pi:
            self.loc.theta=-numpy.pi
        if self.loc.theta < -numpy.pi:
            self.loc.theta= numpy.pi
        #cálculo de coordenada x
        self.loc.x=self.loc.x+self.r*((self.wr+self.wl)/2)*self.dt*cos(self.loc.theta)
        #cálculo de coordenada y
        self.loc.y=self.loc.y+self.r*((self.wr+self.wl)/2)*self.dt*sin(self.loc.theta)

    #Cálculo de errores de distancia y ángulo
    def calculateErrors(self):
        if self.loc.theta >= 0:
            self.etheta=numpy.arctan2(self.goalY,self.goalX)-self.loc.theta
        elif self.loc.theta < 0 :
            self.etheta=-(numpy.arctan2(self.goalY,self.goalX)+self.loc.theta)
        self.ed=numpy.sqrt(math.pow(self.goalX-self.loc.x,2)+math.pow(self.goalY-self.loc.y,2))

    #Cálculo de velocidad con constantes de control P
    def calculateVel(self):
        vel=self.ed*self.kv
        velW=self.etheta*self.kw    
        self.transform.linear.x=vel
        self.transform.angular.z=velW

#creación del objeto puzzlebot
pb=control()

#función para recibir datos de velocidad angular de llanta izquierda
def callWl(data):
    pb.wl= data.data

#función para recibir datos de velocidad angular de llanta derecha
def callWr(data):
    pb.wr=data.data

def scan_callback(scan):
    # Obtener la distancia mínima al obstáculo del scan
    #min_distance = min(scan.ranges)
    pb.front_ranges = scan.ranges[45:135]
    pb.min_distance = min(pb.front_ranges)
    if pb.min_distance < 0.7:
        pb.distance_to_obstacle = pb.min_distance
        pb.obstacle_detected = True
    else:
        pb.distance_to_obstacle = float('inf')
        pb.obstacle_detected = False
    print(pb.obstacle_detected)

#función main
def localize():
    #inicializar el nodo
    rospy.init_node('localization',anonymous=True)
    #crear suscribers para leer velocidad angular
    rospy.Subscriber('wl', Float32, callWl)
    rospy.Subscriber('wr', Float32, callWr)
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    #crear publishers para tópicos
    pubETheta=rospy.Publisher('etheta',Float32,queue_size=10)
    pubED=rospy.Publisher('ed',Float32,queue_size=10)
    pubCoord=rospy.Publisher('coord', Pose2D, queue_size=10)
    pubVel=rospy.Publisher('cmd_vel',Twist,queue_size=10)

    #funciones para obtener dt
    base=rospy.get_time()
    now=rospy.get_time()
    #cálculo de desplazamiento
    desp=numpy.sqrt(math.pow(pb.goalX,2)+math.pow(pb.goalY,2))
    while not rospy.is_shutdown():
        #se actualiza el tiempo actual
        now=rospy.get_time()
        #se calcula dt
        pb.dt=now-base
        #mientras que dt no sea mayor o igual a 0.12, no se hará ningún cálculo
        if pb.dt >= 0.012:
            #se calculan todos lopb.transform.linear.x=0s parámetros
            pb.calculatePosition()
            pb.calculateErrors()
            pb.calculateVel()

            #si hay más de 0.03 radianes de error en el ángulo, se detiene hasta que se alinee con la ruta
            if pb.etheta > 0.03:
                pb.transform.linear.x=0
            #si se encuentra a el 3 por ciento del desplazamiento total de la posición final, se detiene el robot
            if pb.ed < desp*.08:
                pb.transform.linear.x=0
                pb.transform.angular.z=0

            ### meter aqui la parte de lidar
            if pb.obstacle_detected == True:
                pb.transform.linear.x=0
                pb.transform.angular.z=0.1
            print(pb.obstacle_detected)

            ################

            #se publica la información en todos los tópicos
            pubETheta.publish(pb.etheta)
            pubED.publish(pb.ed)
            pubCoord.publish(pb.loc)
            pubVel.publish(pb.transform)
            #se obtiene un nuevo tiempo base para el cálculo de dt
            base=rospy.get_time()

if __name__ == '__main__':
    localize()
