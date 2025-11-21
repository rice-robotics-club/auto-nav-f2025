import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class KeyboardVisualizer(Node):
    def __init__(self):
        super().__init__('keyboard_visualizer')

        # Subscribe to image and keyboard poses
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/keyboard_poses', self.pose_callback, 10
        )

        self.bridge = CvBridge()
        self.key_poses = []  # list of PoseStamped

        # RealSense intrinsics (example 640x480)
        self.fx = 616.0
        self.fy = 616.0
        self.cx = 320.0
        self.cy = 240.0

    def pose_callback(self, msg):
        # Store incoming key poses
        self.key_poses.append(msg)

    def image_callback(self, img_msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')

        # Draw each key
        for pose in self.key_poses:
            x, y, z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z

            if z <= 0:  # avoid division by zero
                continue

            u = int(self.fx * x / z + self.cx)
            v = int(self.fy * y / z + self.cy)

            # Draw a circle and label
            cv2.circle(cv_image, (u, v), 5, (0, 0, 255), -1)
            cv2.putText(cv_image, pose.header.frame_id, (u+5, v-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Show the image
        cv2.imshow("Keyboard Visualization", cv_image)
        cv2.waitKey(1)
        # Clear poses after drawing
        self.key_poses = []

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardVisualizer()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
