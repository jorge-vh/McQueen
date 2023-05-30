#!/usr/bin/env python
# Obtener la posicion, velocidad y covarianza del robot y enviarlo en un msg de odometria
# Equipo #3

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Float32
import tf_conversions
import math
import numpy as np

class odometry:
   def __init__(self):
      # Posicion del robot
      self.x = 0
      self.y = 0
      self.angle = 0
      self.pose = np.zeros((3,1))

      # Velocidades 
      self.v_wl = 0 # Llantas
      self.v_wr = 0 
      self.vel_linear = 0 # Robot
      self.vel_angular = 0
      
      # Definir parametros del sistema
      self.l = 0.18 # longitud
      self.r = 0.05 # radio
      self.k_r = 0.01
      self.k_l = 0.01
      self.sample_time = 0.02 

      # Tiempo
      self.first = True
      self.start_time = 0
      self.current_time = 0
      self.last_time = 0
      self.proc_output = 0

      # Covarianza
      self.sigma_omega = np.eye(2)
      self.nabla_H_omegas = np.ones([3,2])
      self.Q_k = np.zeros([3,3])
      self.H_k = np.eye(3)
      self.Sigma = np.zeros([3,3])
      self.covariance=[0]*36

      # Declarar mensaje de entrada
      self.wr = 0
      self.wl = 0

      # Declarar mensaje de salida
      self.odom = Odometry()
      self.odom.header.stamp = rospy.Time.now()
      self.odom.header.frame_id = "world"
      self.odom.child_frame_id = "base_link"
      self.odom.pose.pose.position.x = 0
      self.odom.pose.pose.position.y = 0
      self.odom.pose.pose.position.z = self.r
      self.odom.pose.pose.orientation.x = 0
      self.odom.pose.pose.orientation.y = 0
      self.odom.pose.pose.orientation.z = 0
      self.odom.pose.pose.orientation.w = 0
      self.odom.pose.covariance = self.covariance
      self.odom.twist.twist.linear.x = 0
      self.odom.twist.twist.linear.y = 0
      self.odom.twist.twist.linear.z = 0
      self.odom.twist.twist.angular.x = 0
      self.odom.twist.twist.angular.y = 0
      self.odom.twist.twist.angular.z = 0
      self.odom.twist.covariance = self.covariance

      # Suscribers
      rospy.Subscriber('/wr',Float32,self.wr_cb)
      rospy.Subscriber('/wl',Float32,self.wl_cb)
      # Publishers
      self.odomet = rospy.Publisher('/odom',Odometry,queue_size=10)

   # Callback wr
   def wr_cb(self, msg):
      self.wr = msg.data

   # Callback wl
   def wl_cb(self, msg):
      self.wl = msg.data

   def wrapToPi(self, angle):
      result = np.fmod((angle+np.pi), (2*np.pi))
      if (result < 0):
         result += 2*np.pi
      return result - np.pi

   def main(self):
      # Definir variables
      if self.first == True:
         self.start_time = rospy.get_time()
         self.last_time = rospy.get_time()
         self.current_time = rospy.get_time()
         self.first = False
      # Sistema
      else:
         # Definir tiempo de muestra
         self.current_time = rospy.get_time()
         dt = self.current_time - self.last_time
         if dt >= self.sample_time:
            # Velocidades de las ruedas
            self.v_wl = self.r * self.wl
            self.v_wr = self.r * self.wr
            # Velocidad del robot
            self.vel_linear = (self.v_wl + self.v_wr)/2
            self.vel_angular = (self.v_wr - self.v_wl)/self.l
            # Calcular covarianza de las ruedas
            self.sigma_omega[0,0] = self.k_r * abs(self.wr)
            self.sigma_omega[1,1] = self.k_l * abs(self.wl)

            self.nabla_H_omegas[0,:] = math.cos(self.angle)
            self.nabla_H_omegas[1,:] = math.sin(self.angle)
            self.nabla_H_omegas[2,0] = 2/self.l
            self.nabla_H_omegas[2,1] = -2/self.l
            self.nabla_H_omegas = (self.r)/2 * self.nabla_H_omegas * dt
            # Calcular Qk
            nabla_t = self.nabla_H_omegas.transpose()
            self.Q_k = np.dot(np.dot(self.nabla_H_omegas,self.sigma_omega),nabla_t)
            # Calcular Hk
            self.H_k[0,2] = -self.vel_linear * math.sin(self.angle) * dt
            self.H_k[1,2] = self.vel_linear * math.cos(self.angle) * dt
            # Obtener sigma
            H_k_t = self.H_k.transpose()
            self.Sigma = np.dot(np.dot(self.H_k, self.Sigma), H_k_t) + self.Q_k

            # Estimar posicion
            self.x += self.vel_linear*math.cos(self.angle) * dt
            self.y += self.vel_linear*math.sin(self.angle) * dt
            self.angle += self.vel_angular * dt
            self.angle = self.wrapToPi(self.angle)

            self.last_time = rospy.get_time()
            
            # Obtener angulo en quaterniones
            self.q = tf_conversions.transformations.quaternion_from_euler(0,0,self.angle)

            # Llenar mensaje de odometria
            self.odom.header.stamp = rospy.Time.now()
            self.odom.pose.pose.position.x = self.x
            self.odom.pose.pose.position.y = self.y
            self.odom.pose.pose.position.z = self.r
            self.odom.pose.pose.orientation.x = self.q[0]
            self.odom.pose.pose.orientation.y = self.q[1]
            self.odom.pose.pose.orientation.z = self.q[2]
            self.odom.pose.pose.orientation.w = self.q[3]
            self.odom.pose.covariance[0] = self.Sigma[0,0]
            self.odom.pose.covariance[1] = self.Sigma[0,1]
            self.odom.pose.covariance[5] = self.Sigma[0,2]
            self.odom.pose.covariance[6] = self.Sigma[1,0]
            self.odom.pose.covariance[7] = self.Sigma[1,1]
            self.odom.pose.covariance[11] = self.Sigma[1,2]
            self.odom.pose.covariance[30] = self.Sigma[2,0]
            self.odom.pose.covariance[31] = self.Sigma[2,1]
            self.odom.pose.covariance[35] = self.Sigma[2,2]
            self.odom.twist.twist.linear.x = self.vel_linear
            self.odom.twist.twist.angular.z = self.vel_angular
            # Mostrar x,y,theta
            print(round(self.odom.pose.pose.position.x,2), round(self.odom.pose.pose.position.y,2), round(self.odom.pose.pose.orientation.z,2))
            self.odomet.publish(self.odom) 

if __name__ == '__main__':
   rospy.init_node('odometry')
   puzzlebot = odometry()
   rate = rospy.Rate(100)
   try:
      while not rospy.is_shutdown():
         puzzlebot.main()
         rate.sleep()
   except rospy.ROSInterruptException:
         pass
