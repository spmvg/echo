import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray

import threading
from queue import LifoQueue, Full, Empty

import pyttsx3
import tempfile
import wave
import numpy as np


# Must match stt_onboard's SAMPLE_RATE
TARGET_SAMPLE_RATE = 16000


class TTSOnboard(Node):
    def __init__(self):
        super().__init__("tts_onboard")
        self.sub = self.create_subscription(
            String, "/tts_onboard/say", self._on_transcript, 10
        )
        # Publish audio to stt_onboard for playback (avoids audio device conflicts)
        self.audio_pub = self.create_publisher(Int16MultiArray, "/stt_onboard/audio_out", 10)
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

    def _resample_audio(self, audio: np.ndarray, from_rate: int, to_rate: int) -> np.ndarray:
        """Resample audio from one sample rate to another using linear interpolation."""
        if from_rate == to_rate:
            return audio
        duration = len(audio) / from_rate
        new_length = int(duration * to_rate)
        indices = np.linspace(0, len(audio) - 1, new_length)
        return np.interp(indices, np.arange(len(audio)), audio.astype(np.float32)).astype(np.int16)

    def _generate_and_publish_audio(self, text: str):
        """
        Generate audio using pyttsx3 and publish to stt_onboard for playback.
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

                # Convert to int16 if needed
                if dtype != np.int16:
                    # Scale to int16 range
                    audio = (audio.astype(np.float32) / np.iinfo(dtype).max * np.iinfo(np.int16).max).astype(np.int16)

                # Convert stereo to mono if needed
                if n_channels > 1:
                    audio = np.reshape(audio, (-1, n_channels))
                    audio = audio.mean(axis=1).astype(np.int16)

                # Resample to target sample rate
                audio = self._resample_audio(audio, framerate, TARGET_SAMPLE_RATE)

                # Publish audio to stt_onboard
                msg = Int16MultiArray()
                msg.data = audio.tolist()
                self.audio_pub.publish(msg)
                self.get_logger().debug(f"Published {len(audio)} audio samples to stt_onboard")

    def _worker_loop(self):
        # Runs in background thread and processes queued text items
        while not self._stop_event.is_set():
            try:
                text = self._queue.get(timeout=0.1)
            except Empty:
                continue

            self._engine.setProperty('rate', self._rate)  # for some reason, this value resets sometimes?
            self._generate_and_publish_audio(text)

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
