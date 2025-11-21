import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np


class KeyboardCenterNode(Node):
    def __init__(self):
        super().__init__("keyboard_center_from_tags")

        self.tag_frames = [
            "aruco_tag_1",
            "aruco_tag_2",
            "aruco_tag_3",
            "aruco_tag_4"
        ]

        self.parent_frame = "camera_A"   # the camera that sees the tags

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.update_center)   # 20 Hz

    def update_center(self):
        positions = []

        for tag in self.tag_frames:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.parent_frame,
                    tag,
                    rclpy.time.Time()
                )

                pos = np.array([
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z
                ])
                positions.append(pos)

            except Exception:
                # If a tag is not seen, skip it
                continue

        if len(positions) < 3:
            self.get_logger().warn("Not enough tags visible to compute keyboard center.")
            return

        # Compute centroid of visible tags
        centroid = np.mean(positions, axis=0)

        # Orientation: average the tag orientations (simple version)
        # You can also compute plane normal if needed
        q = [0, 0, 0, 1]  # identity orientation for now

        # Publish keyboard_center frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = "keyboard_center"

        t.transform.translation.x = float(centroid[0])
        t.transform.translation.y = float(centroid[1])
        t.transform.translation.z = float(centroid[2])

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = KeyboardCenterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
