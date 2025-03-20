import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class MotionControlNode(Node):
    def __init__(self):
        super().__init__('motion_control_node')
        self.publisher = self.create_publisher(Twist, '/key_vel', 10)
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image',
            self.image_callback,
            10)

        self.bridge = CvBridge()
        self.target_detected = False
        self.target_position = None
        self.get_logger().info("Motion Control Node initialized.")

    def move_forward(self, speed=1.0):  # la vitesse pour plus de contrôle
        twist = Twist()
        twist.linear.x = speed
        self.publisher.publish(twist)

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)

    def rotate(self, angular_speed=0.3):  # la vitesse angulaire pour plus de contrôle
        twist = Twist()
        twist.angular.z = angular_speed
        self.publisher.publish(twist)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 🔴 Détection de la canette rouge (pour l'instant)
        red_lower1, red_upper1 = np.array([0, 100, 50]), np.array([10, 255, 255])
        red_lower2, red_upper2 = np.array([170, 100, 50]), np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, red_lower1, red_upper1) + cv2.inRange(hsv, red_lower2, red_upper2)

        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours_red:
            self.get_logger().info("🔴 Canette rouge détectée ! Avance vers elle.")
            largest_contour = max(contours_red, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            self.target_position = (x + w // 2, y + h // 2)
            self.target_detected = True

            #  Seuil de distance (arrêt si assez proche)
            if h > 175:  # Augmenter le seuil pour arrêter plus tôt
                self.stop()
                self.get_logger().info("📍 Arrêt devant la canette.")
    
            else:
                # la direction en fonction de la position de la canette
                frame_center_x = frame.shape[1] // 2
                if self.target_position[0] < frame_center_x - 20:
                    self.rotate(angular_speed=0.3)  # Tourne à gauche
                elif self.target_position[0] > frame_center_x + 20:
                    self.rotate(angular_speed=-0.3)  # Tourne à droite
                else:
                    self.move_forward(speed=0.3)

def main(args=None):
    rclpy.init(args=args)
    node = MotionControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
