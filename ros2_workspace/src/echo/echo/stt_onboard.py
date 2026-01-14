import base64
import json
import os
import queue
import threading
from base64 import b64encode
from io import BytesIO
from queue import LifoQueue
import wave
from time import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice
import numpy as np
import websockets

try:
    import pocketsphinx
    from pocketsphinx import Pocketsphinx, Decoder
except Exception as e:
    raise RuntimeError(
        "Failed to import pocketsphinx. Make sure 'pocketsphinx' is installed.\n"
        "On Debian/Ubuntu: sudo apt install swig build-essential portaudio19-dev\n"
        "then pip install pocketsphinx pyaudio"
    ) from e


WAKE_WORD_MODE = "wakeword"
LANGUAGE_SEARCH_MODE = "lm"
BELL_SOUND = "bell_freesound_116779_creative_commons_0"
BELL_END_SOUND = "bell2_freesound_91924_creative_commons_0"
MODEL = "gpt-4o-realtime-preview"
WS_URL = f"wss://api.openai.com/v1/realtime?model={MODEL}"
SAMPLE_RATE = 16000
CHANNELS = 1
DTYPE = "int16"
CHUNK_SIZE = 4096


def pcm16_to_base64(audio: np.ndarray) -> str:
    return base64.b64encode(audio.tobytes()).decode("utf-8")

def base64_to_pcm16(data: str) -> np.ndarray:
    return np.frombuffer(base64.b64decode(data), dtype=DTYPE)


class STTOnboard(Node):
    """
    A simple ROS2 node that listens to the microphone via PocketSphinx LiveSpeech
    and publishes recognized phrases on /tts_onboard/say (std_msgs/String).
    """

    def __init__(self):
        super().__init__("stt_onboard")
        self.pub = self.create_publisher(String, "/tts_onboard/say", 10)
        self.sounds_pub = self.create_publisher(String, "/sounds/play", 10)
        self.ai_pub = self.create_publisher(String, "/speech_ai/audio", 10)
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        if not self.openai_api_key:
            raise RuntimeError('No OPENAI_API_KEY, cannot proceed.')
        self.ws = None
        self.mode = WAKE_WORD_MODE
        self.audio_out_queue = queue.LifoQueue()

        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("STTOnboard listener started")

    def on_open(self, ws):
        self.get_logger().info("WebSocket connected.")

        session_update = {
            "type": "session.update",
            "session": {
                "modalities": ["audio", "text"],
                "audio": {
                    "input_format": "pcm16",
                    "output_format": "pcm16",
                    "sample_rate": SAMPLE_RATE,
                    "channels": CHANNELS
                },
                "instructions": "You are a concise voice assistant."
            }
        }

        ws.send(json.dumps(session_update))

    @staticmethod
    def on_message(ws, message):
        event = json.loads(message)

        if event["type"] == "response.audio.delta":
            audio = base64_to_pcm16(event["delta"])
            audio_out_queue.put(audio)

        elif event["type"] == "response.audio.done":
            pass

    def on_error(self, ws, error):
        self.get_logger().error("WebSocket error:", error)

    def on_close(self, ws, *_):
        self.get_logger().info("WebSocket closed.")

    def microphone_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warning(status)

        if not self.ws:
            return

        audio_b64 = pcm16_to_base64(indata.copy())

        self.ws.send(json.dumps({
            "type": "input_audio_buffer.append",
            "audio": audio_b64
        }))

    def speaker_callback(self, outdata, frames, time, status):
        if status:
            self.get_logger().warning(status)

        try:
            audio = self.audio_out_queue.get_nowait()
            outdata[:] = audio.reshape(-1, 1)
        except queue.Empty:  # TODO: maybe check size to prevent jitter?
            outdata.fill(0)

    def _listen_loop(
            self,
            wake_word: str = "echo listen",
    ):
        """
        Uses PocketSphinx's built-in keyword spotting to listen for a wake word.
        Once detected, open a socket towards OpenAI.
        """
        audio_q = LifoQueue()
        headers = [
            f"Authorization: Bearer {self.openai_api_key}",
            "OpenAI-Beta: realtime=v1"
        ]

        self.get_logger().info(f"Listening for wake word '{wake_word}'...")

        decoder = make_decoder(wake_word)

        self.get_logger().info('Listing available sound devices:\n'+str(sounddevice.query_devices()))

        input_stream = sounddevice.InputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype=DTYPE,
            blocksize=CHUNK_SIZE,
            callback=self.microphone_callback,
        )

        output_stream = sounddevice.OutputStream(
            samplerate=SAMPLE_RATE,
            channels=CHANNELS,
            dtype=DTYPE,
            blocksize=CHUNK_SIZE,
            callback=self.speaker_callback,
        )

        with input_stream, output_stream:
            decoder.start_utt()

            while True:
                raw = audio_q.get().tobytes()
                decoder.process_raw(raw)

                hyp = decoder.hyp()

                if self.mode == WAKE_WORD_MODE and hyp:
                    self.get_logger().info(f"Wake word ({hyp.hypstr}) detected!")
                    self.mode = LANGUAGE_SEARCH_MODE
                    decoder.end_utt()
                    decoder.set_search(LANGUAGE_SEARCH_MODE)
                    decoder.start_utt()

                    self.ws = websocket.WebSocketApp(
                        WS_URL,
                        header=headers,
                        on_open=self.on_open,
                        on_message=self.on_message,
                        on_error=self.on_error,
                        on_close=self.on_close,
                    )
                    ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
                    ws_thread.start()

                    self.sounds_pub.publish(String(data=BELL_SOUND))
                    continue
                elif self.mode == WAKE_WORD_MODE:
                    continue
                elif self.mode == LANGUAGE_SEARCH_MODE:
                    continue
                else:
                    raise RuntimeError("Unknown mode in STTOnboard listen loop")


def make_decoder(wake_word: str) -> Decoder:
    model_path = os.path.join(os.path.dirname(pocketsphinx.__file__), 'model', 'en-us')

    config = Decoder.default_config()
    config.set_string('-hmm', os.path.join(model_path, 'en-us'))
    config.set_string('-dict', os.path.join(model_path, 'cmudict-en-us.dict'))
    config.set_string('-lm', os.path.join(model_path, 'en-us.lm.bin'))
    decoder = Decoder(config)

    # --- Add wake word search ---
    decoder.set_keyphrase(WAKE_WORD_MODE, wake_word)
    decoder.set_search(WAKE_WORD_MODE)

    # --- Add language model search ---
    decoder.set_lm_file(LANGUAGE_SEARCH_MODE, os.path.join(model_path, 'en-us.lm.bin'))

    return decoder

def main(args=None):
    rclpy.init(args=args)
    node = STTOnboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down STTOnboard node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()