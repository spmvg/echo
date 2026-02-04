from os import getenv

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import requests


class Initialization(Node):
    def __init__(self):
        super().__init__("initialization")
        self.get_logger().info("Starting initialization node")

        self.pub = self.create_publisher(String, "/tts_onboard/say", 10)

        # Wait for both tts_onboard and stt_onboard nodes to be available
        # stt_onboard handles audio playback, tts_onboard generates audio
        self._wait_for_node("tts_onboard", timeout_sec=30.0)
        self._wait_for_node("stt_onboard", timeout_sec=30.0)
        self.pub.publish(String(data="Power on"))

        try:
            requests.get("https://www.google.com", timeout=5)
            self.get_logger().info("Internet connected")
        except:
            self.get_logger().warning("No internet connection")
            self.pub.publish(String(data="No internet connection"))

        openai_api_key = getenv("OPENAI_API_KEY")
        if openai_api_key:
            self.get_logger().info("AI API key configured")
        else:
            self.get_logger().warning("Missing AI API key")
            self.pub.publish(String(data="Missing AI API key"))

        prompt = getenv("PROMPT")
        if prompt:
            self.get_logger().info(f"Prompt configured: {prompt}")
            self.pub.publish(String(data="Custom prompt configured"))
        else:
            self.pub.publish(String(data="Using default prompt"))

    def _wait_for_node(self, node_name: str, timeout_sec: float = 30.0):
        """Wait for a node to become available using ROS2 graph API."""
        self.get_logger().info(f"Waiting for node '{node_name}' to be available...")
        start_time = self.get_clock().now()
        while rclpy.ok():
            node_names = [name for name, _ in self.get_node_names_and_namespaces()]
            if node_name in node_names:
                self.get_logger().info(f"Node '{node_name}' is now available")
                return True

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warning(f"Timeout waiting for node '{node_name}'")
                return False

            rclpy.spin_once(self, timeout_sec=0.5)


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
