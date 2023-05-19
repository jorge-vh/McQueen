import cv2
import cv2.aruco as aruco

# Cargar una imagen o capturar un cuadro de la cámara
frame = cv2.imread('image.png')

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
cv2.waitKey(0)
cv2.destroyAllWindows()
