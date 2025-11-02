import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
from queue import LifoQueue, Full, Empty

import pyttsx3



class TTSOnboard(Node):
    def __init__(self):
        super().__init__("tts_onboard")
        self.sub = self.create_subscription(
            String, "/tts_onboard/say", self._on_transcript, 10
        )
        self.get_logger().info("TTSOnboard subscriber started, listening on /tts_onboard/say")

        # Queue and worker used to avoid blocking the ROS callback while speaking
        self._queue: LifoQueue = LifoQueue(maxsize=32)
        self._stop_event = threading.Event()
        self._engine = None

        self._engine = pyttsx3.init()
        self._rate = 130
        self._engine.setProperty('rate', self._rate)

        # Start background thread which will consume the queue and call the engine
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info("pyttsx3 engine initialized and worker started")

    def _on_transcript(self, msg: String):
        self.get_logger().info(f"Speaking: {msg.data}")

        try:
            self._queue.put_nowait(msg.data)
        except Full:
            # Queue is bounded to avoid unbounded memory growth
            self.get_logger().warning("TTS queue full - dropping incoming message")

    def _worker_loop(self):
        # Runs in background thread and processes queued text items
        while not self._stop_event.is_set():
            try:
                text = self._queue.get(timeout=0.1)
            except Empty:
                continue

            self._engine.setProperty('rate', self._rate)  # for some reason, this value resets sometimes?
            self._engine.say(text)
            self._engine.runAndWait()


    def destroy_node(self):
        # Gracefully stop the worker and engine before destroying the node
        self.get_logger().info("Stopping TTS worker and shutting down engine")
        self._stop_event.set()
        self._worker.join(timeout=1.0)
        self._engine.stop()

        return super().destroy_node()


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
