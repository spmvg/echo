#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# PocketSphinx LiveSpeech import (from pocketsphinx package)
try:
    from pocketsphinx import LiveSpeech
except Exception as e:
    raise RuntimeError(
        "Failed to import pocketsphinx. Make sure 'pocketsphinx' is installed.\n"
        "On Debian/Ubuntu: sudo apt install swig build-essential portaudio19-dev\n"
        "then pip install pocketsphinx pyaudio"
    ) from e


class PocketSphinxNode(Node):
    """
    A simple ROS2 node that listens to the microphone via PocketSphinx LiveSpeech
    and publishes recognized phrases on /speech_to_text/transcript (std_msgs/String).
    """

    def __init__(self):
        super().__init__("pocketsphinx_listener")
        self.pub = self.create_publisher(String, "/speech_to_text/transcript", 10)

        # You can tune LiveSpeech arguments to change language model, keyphrase, thresholds, etc.
        # By default this listens continuously and produces phrases as they are detected.
        self.speech = LiveSpeech(
            # sampling_rate=16000,  # default is usually 16000
            # buffer_size=2048,
            # no_search=False,
            # full_utt=False,
            # If you want keyword spotting: lm=False, keyphrase='hey robot', kws_threshold=1e-20
        )

        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("PocketSphinx listener started")

    def _listen_loop(self):
        # The LiveSpeech iterator blocks and yields recognized chunks.
        for phrase in self.speech:
            try:
                text = str(phrase).strip()
                if not text:
                    continue
                msg = String()
                msg.data = text
                # publish on the ROS topic
                self.pub.publish(msg)
                self.get_logger().info(f"Recognized: {text}")
            except Exception as e:
                self.get_logger().error(f"Error during recognition loop: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PocketSphinxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down pocketsphinx_listener node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()