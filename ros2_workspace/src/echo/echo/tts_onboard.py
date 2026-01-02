import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
from queue import LifoQueue, Full, Empty

import pyttsx3
import time
import tempfile
import wave
import numpy as np
import sounddevice as sd



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

    def _speak_to_sounddevice(self, text: str):
        """
        Hack around pyttsx3 not raising if device is unavailable by writing to temp WAV and playing via sounddevice.
        """
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=True) as tf:
            self._engine.save_to_file(text, tf.name)
            self._engine.runAndWait()

            with wave.open(tf.name, "rb") as wf:
                n_channels = wf.getnchannels()
                sampwidth = wf.getsampwidth()
                framerate = wf.getframerate()
                n_frames = wf.getnframes()

                # Read audio frames and convert to NumPy array
                data = wf.readframes(n_frames)
                dtype_map = {1: np.int8, 2: np.int16, 4: np.int32}
                dtype = dtype_map.get(sampwidth, np.int16)
                audio = np.frombuffer(data, dtype=dtype)

                # Reshape for stereo or mono
                if n_channels > 1:
                    audio = np.reshape(audio, (-1, n_channels))

                # Normalize to float32 range [-1.0, 1.0]
                audio = audio.astype(np.float32) / np.iinfo(dtype).max

                # Retry loop: try to play, sleeping 500 ms between attempts
                attempt = 0
                while attempt <= 10:
                    try:
                        sd.play(audio, framerate)
                        sd.wait()
                        break
                    except Exception as e:
                        # Log the error and retry after a short sleep
                        self.get_logger().warning(f"Cannot play text-to-speech: {e} - retrying in 500 ms")
                        time.sleep(0.5)

                    attempt += 1

    def _worker_loop(self):
        # Runs in background thread and processes queued text items
        while not self._stop_event.is_set():
            try:
                text = self._queue.get(timeout=0.1)
            except Empty:
                continue

            self._engine.setProperty('rate', self._rate)  # for some reason, this value resets sometimes?
            self._speak_to_sounddevice(text)


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
