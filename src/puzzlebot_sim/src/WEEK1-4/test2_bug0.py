import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Bug0:
    def __init__(self):
        rospy.init_node('bug0', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.goal = [5, 5] # Meta
        self.robot_pose = [0, 0, 0] # Pose actual del robot [x, y, theta]
        self.distance_to_goal = 0 # Distancia al objetivo
        self.min_distance = 0.3 # Distancia mínima de seguridad
        self.wall_following = False # Indica si está siguiendo una pared
        self.prev_distance = 0 # Distancia anterior al objetivo
        self.angle_tolerance = 0.05 # Tolerancia de orientación en radianes
        self.twist = Twist()

    def lidar_callback(self, data):
        # Si está siguiendo una pared, no realiza corrección en la orientación
        if self.wall_following:
            return
        # Obtiene los datos de distancia del lidar
        front_distances = data.ranges[:5] + data.ranges[-5:]
        left_distances = data.ranges[90:180]
        right_distances = data.ranges[-180:-90]
        # Obtiene la distancia al obstáculo más cercano en cada dirección
        front_min_distance = min(front_distances)
        left_min_distance = min(left_distances)
        right_min_distance = min(right_distances)
        # Si no hay obstáculos en el frente, se corrige la orientación hacia el objetivo
        if front_min_distance > self.min_distance:
            angle_to_goal = self.get_angle_to_goal()
            if abs(angle_to_goal) > self.angle_tolerance:
                self.twist.angular.z = 0.5 if angle_to_goal > 0 else -0.5
            else:
                self.twist.angular.z = 0
        # Si hay obstáculo en el frente, se sigue la pared
        else:
            self.wall_following = True
            # Si hay pared a la izquierda, sigue la pared a la izquierda
            if left_min_distance < self.min_distance:
                self.twist.angular.z = 0.5
            # Si hay pared a la derecha, sigue la pared a la derecha
            elif right_min_distance < self.min_distance:
                self.twist.angular.z = -0.5
            # Si no hay pared a ningún lado, se gira en el lugar para buscar una pared
            else:
                self.twist.angular.z = 0.5

    def odom_callback(self, data):
        # Obtiene la pose actual del robot en coordenadas (x, y, theta)
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        _, _, theta = euler_from_quaternion(
            [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
             data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        self.robot_pose = [x, y, theta]
        self.distance_to_goal = ((x - self.goal[0]) ** 2 + (y - self.goal[1]) ** 2) ** 0.5

    def get_angle_to
