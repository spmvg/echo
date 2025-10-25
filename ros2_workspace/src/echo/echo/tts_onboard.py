import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TTSOnboard(Node):
    def __init__(self):
        super().__init__("tts_onboard")
        self.sub = self.create_subscription(
            String, "/tts_onboard/say", self._on_transcript, 10
        )
        self.get_logger().info("TTSOnboard subscriber started, listening on /tts_onboard/say")

    def _on_transcript(self, msg: String):
        # Placeholder: log the transcript. Replace this with actual processing later.
        text = msg.data if msg is not None else ""
        if not text:
            self.get_logger().debug("Received empty transcript")
            return
        self.get_logger().info(f"Transcript received: {text}")


def main(args=None):
    rclpy.init(args=args)
    node = TTSOnboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down TTSOnboard node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

