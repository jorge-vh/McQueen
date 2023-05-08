#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan

def callback(scan):
    # Procesa los datos del lidar
    #front = scan.ranges[len(scan.ranges)//2]
    #left = min(scan.ranges[:len(scan.ranges)//4])
    #right = min(scan.ranges[-len(scan.ranges)//4:])

    right =  min(min(scan.ranges[0:143]), 10)
    frontright = min(min(scan.ranges[144:287]), 10)
    front =  min(min(scan.ranges[288:431]), 10)
    frontleft =  min(min(scan.ranges[432:575]), 10)
    left =   min(min(scan.ranges[576:719]), 10)

    # Determina la posición de los obstáculos
    if front < 1.0:
        print("Obstacle in front")
    if left < 1.0:
        print("Obstacle on the left")
    if right < 1.0:
        print("Obstacle on the right")

def main():
    rospy.init_node('obstacle_detector', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
