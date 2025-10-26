import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import requests


class Initialization(Node):
    def __init__(self):
        super().__init__("initialization")
        self.get_logger().info("Starting initialization node")

        self.pub = self.create_publisher(String, "/tts_onboard/say", 10)

        self.pub.publish("Power on")

        try:
            requests.get("https://www.google.com", timeout=5)
            self.get_logger().info("Internet connected")
            self.pub.publish("Internet connected")
        except:
            self.get_logger().warning("No internet connection")
            self.pub.publish("No internet connection")


def main(args=None):
    rclpy.init(args=args)
    node = Initialization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Initialization node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
