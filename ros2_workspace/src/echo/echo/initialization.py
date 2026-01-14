from os import getenv
from time import sleep

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import requests


class Initialization(Node):
    def __init__(self):
        super().__init__("initialization")
        self.get_logger().info("Starting initialization node")

        self.pub = self.create_publisher(String, "/tts_onboard/say", 10)

        sleep(5)  # wait for tts node to be started
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
