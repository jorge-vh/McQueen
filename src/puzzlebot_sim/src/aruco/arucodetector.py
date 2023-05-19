#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import numpy as np
import cv2.aruco as aruco

class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        
    def detect_aruco_markers(self):
        # Inicializar la cámara
        cap = cv2.VideoCapture(0)  # Índice 0 para la cámara predeterminada
        
        while not rospy.is_shutdown():
            # Capturar el cuadro actual de la cámara
            ret, frame = cap.read()
            
            if not ret:
                rospy.logerr("No se puede capturar el cuadro de la cámara")
                break
            
            # Convertir la imagen a escala de grises
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Definir el diccionario de ArUCo
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            
            # Definir los parámetros de detección
            parameters = aruco.DetectorParameters_create()
            
            # Detectar marcadores ArUCo en la imagen
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            
            # Dibujar los marcadores encontrados en la imagen
            frame_markers = aruco.drawDetectedMarkers(frame, corners, ids)
            
            # Mostrar la imagen con los marcadores
            cv2.imshow("ArUCo Detection", frame_markers)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # Liberar los recursos
        cap.release()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('aruco_detector')
    detector = ArucoDetector()
    detector.detect_aruco_markers()
    rospy.spin()

if __name__ == '__main__':
    main()
