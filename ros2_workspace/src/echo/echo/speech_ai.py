from base64 import b64decode

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
from queue import LifoQueue, Full, Empty



class SpeechAI(Node):
    def __init__(self):
        super().__init__("speech_ai")
        self.sub = self.create_subscription(
            String, "/speech_ai/audio", self._on_transcript, 10
        )
        self.get_logger().info("SpeechAI subscriber started, listening on /speech_ai/audio")

        # Queue and worker used to avoid blocking the ROS callback while speaking
        self._queue: LifoQueue = LifoQueue(maxsize=32)
        self._stop_event = threading.Event()

        # Start background thread which will consume the queue and call the engine
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info("SpeechAI worker started")

    def _on_transcript(self, msg: String):
        try:
            self._queue.put_nowait(b64decode(msg.data.encode()))
        except Full:
            # Queue is bounded to avoid unbounded memory growth
            self.get_logger().warning("SpeechAI queue full - dropping incoming message")

    def _worker_loop(self):
        # Runs in background thread and processes queued text items
        while not self._stop_event.is_set():
            try:
                bytes_text = self._queue.get(timeout=0.1)

                self.get_logger().info(f"Querying OpenAI with {len(bytes_text)} bytes")
                # TODO
            except Empty:
                continue

    def destroy_node(self):
        # Gracefully stop the worker and engine before destroying the node
        self.get_logger().info("Stopping SpeechAI worker and shutting down engine")
        self._stop_event.set()
        self._worker.join(timeout=1.0)

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpeechAI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down SpeechAI node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
