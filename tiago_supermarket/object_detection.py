import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time  

class CanDetectionNode(Node):
    def __init__(self):
        super().__init__('can_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/head_front_camera/image',  
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.last_time = time.time()  

        
        cv2.namedWindow("Détection de canettes", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Masque Rouge", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Masque Vert", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        if time.time() - self.last_time < 0.5:
            return
        self.last_time = time.time()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 🔴 Seuils pour le rouge
            red_lower1, red_upper1 = np.array([0, 100, 50]), np.array([10, 255, 255])
            red_lower2, red_upper2 = np.array([170, 100, 50]), np.array([180, 255, 255])
            mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
            mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
            mask_red = cv2.add(mask_red1, mask_red2)  

            # 🟢 Seuils pour le vert
            green_lower, green_upper = np.array([30, 40, 40]), np.array([90, 255, 255])
            mask_green = cv2.inRange(hsv, green_lower, green_upper)

            # 📌 Affichage des masques
            cv2.imshow("Masque Rouge", mask_red)
            cv2.imshow("Masque Vert", mask_green)
            cv2.waitKey(1)  

            # 📏 Détection des contours
            contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            self.get_logger().info(f"🔴 {len(contours_red)} contours rouges trouvés")
            self.get_logger().info(f"🟢 {len(contours_green)} contours verts trouvés")

            #  Debug 
            debug_frame = frame.copy()
            cv2.drawContours(debug_frame, contours_red, -1, (0, 0, 255), 1)
            cv2.drawContours(debug_frame, contours_green, -1, (0, 255, 0), 1)
            cv2.imshow("Contours Debug", debug_frame)

            # Dessine les rectangles seulement pour les gros objets
            for contour in contours_red:
                if cv2.contourArea(contour) > 100:  # Réduction du seuil
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    self.get_logger().info(f"🔴 Canette rouge détectée à : X={x}, Y={y}")

            for contour in contours_green:
                if cv2.contourArea(contour) > 100:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    self.get_logger().info(f"🟢 Canette verte détectée à : X={x}, Y={y}")

            # Affichage de l’image originale avec les contours
            cv2.imshow("Détection de canettes", frame)
            cv2.waitKey(1)

            # Quitte proprement avec la touche 'q'
            if cv2.waitKey(10) & 0xFF == ord('q'):
                self.get_logger().info("Fermeture demandée par l'utilisateur.")
                cv2.destroyAllWindows()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Erreur conversion image : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CanDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()  

if __name__ == '__main__':
    main()
