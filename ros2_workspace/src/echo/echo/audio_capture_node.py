import threading
from base64 import b64encode
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice

SAMPLING_RATE = 44100
DEFAULT_BLOCKSIZE = 44100
DEFAULT_CHANNELS = 1


class AudioCaptureNode(Node):
    """A small ROS2 node that captures microphone audio and publishes base64 chunks to /audio/raw.

    This decouples audio capture from speech processing so the capture can run in a separate
    process and won't be blocked by CPU work in the recognizer.
    """

    def __init__(self,
                 topic: str = "/audio/raw",
                 samplerate: int = SAMPLING_RATE,
                 channels: int = DEFAULT_CHANNELS,
                 blocksize: int = DEFAULT_BLOCKSIZE,
                 dtype: str = "int16"):
        super().__init__("audio_capture")
        self.pub = self.create_publisher(String, topic, 100)
        self.samplerate = samplerate
        self.channels = channels
        self.blocksize = blocksize
        self.dtype = dtype

        # Use a background thread to hold the sounddevice stream alive while rclpy.spin runs
        self._stream_thread = threading.Thread(target=self._run_stream, daemon=True)
        self._stream_thread.start()
        self.get_logger().info(f"AudioCaptureNode started: publishing {topic} @ {samplerate}Hz")

    def _run_stream(self):
        def audio_callback(indata, frames, callback_time, status):
            # Keep the audio callback minimal and log status if present
            if status:
                self.get_logger().error(f"Audio callback status: {status}")

            # Publish base64-encoded raw PCM bytes
            self.get_logger().info(f"Captured {frames} frames")
            payload = b64encode(indata.copy().tobytes()).decode()
            self.get_logger().info(f"Publishing audio chunk of size {len(payload)}")
            msg = String()
            msg.data = payload
            self.pub.publish(msg)
            self.get_logger().info("Published audio chunk")

        # Open the input stream and keep it running; allow exceptions to surface
        with sounddevice.InputStream(
            samplerate=self.samplerate,
            channels=self.channels,
            dtype=self.dtype,
            blocksize=self.blocksize,
            callback=audio_callback,
        ) as stream:
            # Keep the thread alive while the node runs; rclpy.spin will run in the main thread.
            while rclpy.ok():
                time.sleep(1000)


def main(args=None):
    rclpy.init(args=args)
    node = AudioCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down AudioCaptureNode")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
