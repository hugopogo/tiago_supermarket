import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class ArUcoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image',
            self.image_callback,
            10)
        self.bridge = CvBridge()

        # le dictionnaire ArUco prédéfini
        self.aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        self.parameters = cv.aruco.DetectorParameters_create()
        self.get_logger().info("ArUco Detection Node initialized.")

    def image_callback(self, msg):
        self.get_logger().info("Image received.")
        try:
            # le message d'image ROS en image OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # niveaux de gris pour la détection
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

            # Détection des marqueurs ArUco

            corners, ids, _ = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters = self.parameters)

            # Dessiner les boîtes autour des marqueurs détectés
            if ids is not None:
                cv.aruco.drawDetectedMarkers(frame, corners, ids)

                for i in range(len(ids)):
                    x, y = int(corners[i][0][0][0]), int(corners[i][0][0][1])
                    cv.putText(frame, f"ID: {ids[i][0]}", (x, y - 10),
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # Afficher l'image avec les marqueurs détectés
            cv.imshow('ArUco Detection', frame)
            cv.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
