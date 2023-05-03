#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class ObstacleDetection:
    
    def __init__(self):
        # Configurar el nodo de ROS
        rospy.init_node('obstacle_detection')
        
        # Suscribirse al tópico de LaserScan
        rospy.Subscriber('/scan', LaserScan, self.callback)
        
        # Publicar en el tópico de String
        self.pub = rospy.Publisher('/obstacle_location', String, queue_size=10)
    
    def callback(self, data):
        # Obtener los valores del rango de medición del LiDAR
        ranges = data.ranges
        
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
