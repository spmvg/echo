from base64 import b64decode
from io import BytesIO
from os import getenv

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import threading
from queue import LifoQueue, Full, Empty

from openai import OpenAI


class SpeechAI(Node):
    def __init__(self):
        super().__init__("speech_ai")
        self.sub = self.create_subscription(
            String, "/speech_ai/audio", self.on_audio, 10
        )
        self.pub = self.create_publisher(String, "/tts_onboard/say", 10)
        self.get_logger().info("SpeechAI subscriber started, listening on /speech_ai/audio")

        # Queue and worker used to avoid blocking the ROS callback while speaking
        self._queue: LifoQueue = LifoQueue(maxsize=32)
        self._stop_event = threading.Event()

        # Start background thread which will consume the queue and call the engine
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def on_audio(self, msg: String):
        try:
            self._queue.put_nowait(msg.data)
        except Full:
            # Queue is bounded to avoid unbounded memory growth
            self.get_logger().warning("SpeechAI queue full - dropping incoming message")

    def _worker_loop(self):
        client = OpenAI()

        self.get_logger().info("SpeechAI worker started")
        while not self._stop_event.is_set():
            try:
                base64_wav = self._queue.get(timeout=0.1)
            except Empty:
                continue

            self.get_logger().info(f"Querying OpenAI transcript with {len(base64_wav)} bytes")
            bytesio_wav = BytesIO(b64decode(base64_wav))
            bytesio_wav.name = "speech.wav"
            # TODO: use realtime API instead
            transcript = client.audio.transcriptions.create(
                model="whisper-1",
                file=bytesio_wav,
                language="en",
            )
            self.get_logger().info(f"Transcription: {transcript.text}")
            self.pub.publish(String(data=f'You said: {transcript.text}. Let me think...'))

            prompt = getenv("PROMPT") or f"You are a helpful assistant. Your personality is: helpful, creative, clever, and friendly."
            response = client.responses.create(
                model="gpt-5-mini",
                input=[
                    {
                        "role": "user",
                        "content": [
                            # TODO: this should be a system prompt, not a user prompt
                            {"type": "input_text", "text": f"{prompt}\n\nThe input should be extremely short (1 sentence only and keep compound sentences to a minimum). Your response will be read aloud in text-to-speech, so make sure your response sounds like speech. Respond to the user input accordingly."},
                            {"type": "input_text", "text": transcript.text}
                        ]
                    }
                ]
            )
            response_text = response.output_text
            self.get_logger().info(f"OpenAI response: {response_text}")

            self.pub.publish(String(data=response_text))

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
