import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SpeechLogger(Node):
    """
    A simple ROS2 node that subscribes to /speech_to_text/transcript (std_msgs/String)
    and logs received transcripts. Acts as a placeholder for downstream processing.
    """

    def __init__(self):
        super().__init__("speech_logger")
        self.sub = self.create_subscription(
            String, "/speech_to_text/transcript", self._on_transcript, 10
        )
        self.get_logger().info("SpeechLogger subscriber started, listening on /speech_to_text/transcript")

    def _on_transcript(self, msg: String):
        # Placeholder: log the transcript. Replace this with actual processing later.
        text = msg.data if msg is not None else ""
        if not text:
            self.get_logger().debug("Received empty transcript")
            return
        self.get_logger().info(f"Transcript received: {text}")


def main(args=None):
    rclpy.init(args=args)
    node = SpeechLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down SpeechLogger node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

