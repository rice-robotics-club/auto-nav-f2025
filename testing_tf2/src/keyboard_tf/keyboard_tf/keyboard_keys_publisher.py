import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml


class KeyboardKeysPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_keys_publisher')

        # Parameters
        self.declare_parameter("yaml_path", "")
        self.declare_parameter("parent_frame", "keyboard_center")
        yaml_path = self.get_parameter("yaml_path").get_parameter_value().string_value
        parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value

        if yaml_path == "":
            self.get_logger().error("Must supply yaml_path parameter pointing to keyboard_keys.yaml")
            return

        # Load YAML
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        if "keys" not in data:
            self.get_logger().error("YAML file must contain a 'keys:' map.")
            return

        keys = data["keys"]
        self.get_logger().info(f"Loaded {len(keys)} keys from YAML file.")

        # TF broadcaster
        broadcaster = StaticTransformBroadcaster(self)
        transforms = []

        # Create TF for each key
        for key_name, coords in keys.items():
            x = float(coords["x"])
            y = float(coords["y"])
            z = 0.0  # keyboard is flat

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent_frame
            t.child_frame_id = f"key_{self.sanitize(key_name)}"

            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            transforms.append(t)

        # Publish all TFs
        broadcaster.sendTransform(transforms)
        self.get_logger().info("Published all keyboard key transforms.")

    def sanitize(self, key_name: str) -> str:
        """Sanitize frame names to avoid invalid TF characters."""
        return (
            key_name.replace("'", "quote")
                    .replace(";", "semicolon")
                    .replace(",", "comma")
                    .replace(".", "dot")
                    .replace("/", "slash")
                    .replace("[", "lbracket")
                    .replace("]", "rbracket")
                    .replace("\\", "backslash")
                    .replace("-", "minus")
                    .replace("=", "equals")
                    .replace(" ", "_")
        )


def main():
    rclpy.init()
    node = KeyboardKeysPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
