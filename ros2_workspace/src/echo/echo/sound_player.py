import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
from queue import LifoQueue, Full, Empty
import wave

import sounddevice as sd
import numpy as np
import os


class SoundPlayer(Node):
    def __init__(self):
        super().__init__("sound_player")

        # Subscribe to /sounds/play topic
        self.sub = self.create_subscription(
            String, "/sounds/play", self._on_sound_request, 10
        )
        self.get_logger().info("SoundPlayer node started, listening on /sounds/play")

        # Queue + worker thread
        self._queue: LifoQueue = LifoQueue(maxsize=32)
        self._stop_event = threading.Event()

        self._worker = threading.Thread(
            target=self._worker_loop,
            daemon=True,
        )
        self._worker.start()

    def _on_sound_request(self, msg: String):
        wav_type = msg.data.strip()

        if wav_type not in {
            "bell_freesound_116779_creative_commons_0",
            "bell2_freesound_91924_creative_commons_0",
        }:
            return

        wav_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "sounds",
            f"{wav_type}.wav"
        )

        try:
            self._queue.put_nowait(wav_file)
        except Full:
            self.get_logger().warning("Sound queue full - dropping request")

    def _play_wav(self, wav_path: str):
        with wave.open(wav_path, "rb") as wf:
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

            try:
                sd.play(audio, framerate)
                sd.wait()
            except Exception as e:
                self.get_logger().error(f"Error playing sound {wav_path}: {e}")

    def _worker_loop(self):
        # Plays queued wav files
        while not self._stop_event.is_set():
            try:
                wav_path = self._queue.get(timeout=0.1)
            except Empty:
                continue

            self.get_logger().info(f"Playing {wav_path}")
            self._play_wav(wav_path)

    def destroy_node(self):
        self.get_logger().info("Stopping SoundPlayer worker thread")
        self._stop_event.set()
        self._worker.join(timeout=1.0)

        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SoundPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down SoundPlayer node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
