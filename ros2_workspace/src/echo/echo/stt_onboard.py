import asyncio
import base64
import json
import os
import queue
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice
import numpy as np
import websockets

try:
    import pocketsphinx
    from pocketsphinx import Decoder
except Exception as e:
    raise RuntimeError(
        "Failed to import pocketsphinx. Make sure 'pocketsphinx' is installed.\n"
        "On Debian/Ubuntu: sudo apt install swig build-essential portaudio19-dev\n"
        "then pip install pocketsphinx pyaudio"
    ) from e


WAKE_WORD_MODE = "wakeword"
CONVERSATION_MODE = "conversation"
BELL_SOUND = "bell_freesound_116779_creative_commons_0"
BELL_END_SOUND = "bell2_freesound_91924_creative_commons_0"
MODEL = "gpt-4o-realtime-preview"
WS_URL = f"wss://api.openai.com/v1/realtime?model={MODEL}"
SAMPLE_RATE = 16000  # 16kHz to ease load on RPi
CHANNELS = 1
DTYPE = "int16"
CHUNK_SIZE = 4096  # Larger chunks to reduce callback frequency on RPi
SILENCE_TIMEOUT = 10.0  # Seconds of silence before returning to wake word mode


def pcm16_to_base64(audio: np.ndarray) -> str:
    return base64.b64encode(audio.tobytes()).decode("utf-8")

def base64_to_pcm16(data: str) -> np.ndarray:
    return np.frombuffer(base64.b64decode(data), dtype=DTYPE)


class STTOnboard(Node):
    """
    A ROS2 node that listens for a wake word via PocketSphinx, then streams
    audio to OpenAI's realtime voice API via WebSocket for conversation.
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
        self.ws_connected = False
        self.mode = WAKE_WORD_MODE
        self.audio_out_queue = queue.Queue()
        self.audio_in_queue = queue.Queue()
        self.lock = threading.Lock()
        self.loop = None

        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("STTOnboard listener started")

    async def ws_send(self, data: dict):
        """Send data to WebSocket if connected."""
        with self.lock:
            if self.ws and self.ws_connected:
                try:
                    await self.ws.send(json.dumps(data))
                except Exception as e:
                    self.get_logger().error(f"Error sending to WebSocket: {e}")

    async def ws_handler(self):
        """Handle WebSocket connection and messages."""
        headers = {
            "Authorization": f"Bearer {self.openai_api_key}",
            "OpenAI-Beta": "realtime=v1"
        }

        try:
            async with websockets.connect(WS_URL, extra_headers=headers) as ws:
                self.get_logger().info("WebSocket connected.")
                with self.lock:
                    self.ws = ws
                    self.ws_connected = True

                # Send session configuration
                session_update = {
                    "type": "session.update",
                    "session": {
                        "modalities": ["audio", "text"],
                        "input_audio_format": "pcm16",
                        "output_audio_format": "pcm16",
                        "input_audio_transcription": {
                            "model": "whisper-1"
                        },
                        "turn_detection": {
                            "type": "server_vad",
                            "threshold": 0.5,
                            "prefix_padding_ms": 300,
                            "silence_duration_ms": 500
                        },
                        "instructions": "You are a concise voice assistant named Echo. Keep responses brief and helpful."
                    }
                }
                await ws.send(json.dumps(session_update))

                # Listen for messages
                async for message in ws:
                    await self.handle_message(message)

        except websockets.exceptions.ConnectionClosed as e:
            self.get_logger().info(f"WebSocket closed: {e}")
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")
        finally:
            with self.lock:
                self.ws = None
                self.ws_connected = False
            self.get_logger().info("WebSocket disconnected.")

    async def handle_message(self, message: str):
        """Handle incoming WebSocket messages."""
        event = json.loads(message)
        event_type = event.get("type", "")

        if event_type == "response.audio.delta":
            audio = base64_to_pcm16(event["delta"])
            self.audio_out_queue.put(audio)

        elif event_type == "response.audio.done":
            self.get_logger().debug("Audio response complete")

        elif event_type == "response.done":
            self.get_logger().info("Response complete")

        elif event_type == "conversation.item.input_audio_transcription.completed":
            transcript = event.get("transcript", "")
            self.get_logger().info(f"User said: {transcript}")

        elif event_type == "error":
            self.get_logger().error(f"API error: {event.get('error', {})}")

        elif event_type == "session.created":
            self.get_logger().info("Session created")

        elif event_type == "session.updated":
            self.get_logger().info("Session updated")

    def send_audio_to_ws(self, audio_data: np.ndarray):
        """Send audio data to WebSocket if connected."""
        if self.loop and self.ws_connected:
            audio_b64 = pcm16_to_base64(audio_data)
            asyncio.run_coroutine_threadsafe(
                self.ws_send({"type": "input_audio_buffer.append", "audio": audio_b64}),
                self.loop
            )

    def microphone_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warning(f"Mic status: {status}")

        # Always put audio in queue for wake word detection
        self.audio_in_queue.put(indata.copy())

    def speaker_callback(self, outdata, frames, time_info, status):
        if status:
            self.get_logger().warning(f"Speaker status: {status}")

        try:
            audio = self.audio_out_queue.get_nowait()
            # Handle size mismatch
            if len(audio) >= frames:
                outdata[:] = audio[:frames].reshape(-1, 1)
            else:
                outdata[:len(audio)] = audio.reshape(-1, 1)
                outdata[len(audio):] = 0
        except queue.Empty:
            outdata.fill(0)

    def start_conversation(self):
        """Start a WebSocket connection for conversation."""
        def run_async_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self.ws_handler())
            self.loop.close()
            self.loop = None

        ws_thread = threading.Thread(target=run_async_loop, daemon=True)
        ws_thread.start()

        self.sounds_pub.publish(String(data=BELL_SOUND))
        self.mode = CONVERSATION_MODE
        self.get_logger().info("Started conversation mode")

    def end_conversation(self):
        """End the current conversation and return to wake word mode."""
        with self.lock:
            if self.ws:
                # Close the WebSocket from the async loop
                if self.loop:
                    asyncio.run_coroutine_threadsafe(self.ws.close(), self.loop)
                self.ws = None

        self.ws_connected = False
        self.mode = WAKE_WORD_MODE

        # Clear audio queues
        while not self.audio_out_queue.empty():
            try:
                self.audio_out_queue.get_nowait()
            except queue.Empty:
                break

        self.sounds_pub.publish(String(data=BELL_END_SOUND))
        self.get_logger().info("Ended conversation, returning to wake word mode")

    def _listen_loop(self, wake_word: str = "echo listen"):
        """
        Main loop: listen for wake word, then stream audio to OpenAI realtime API.
        """
        self.get_logger().info(f"Listening for wake word '{wake_word}'...")

        decoder = make_decoder(wake_word)

        self.get_logger().info('Available sound devices:\n' + str(sounddevice.query_devices()))

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
                try:
                    # Get audio from the queue (with timeout to allow mode checks)
                    audio_data = self.audio_in_queue.get(timeout=0.1)
                except queue.Empty:
                    continue

                if self.mode == WAKE_WORD_MODE:
                    # Feed audio to PocketSphinx for wake word detection
                    raw = audio_data.tobytes()
                    decoder.process_raw(raw, False, False)

                    hyp = decoder.hyp()
                    if hyp:
                        self.get_logger().info(f"Wake word detected: {hyp.hypstr}")
                        decoder.end_utt()
                        decoder.start_utt()
                        self.start_conversation()

                elif self.mode == CONVERSATION_MODE:
                    # Send audio to OpenAI WebSocket
                    self.send_audio_to_ws(audio_data)

                    # Check if WebSocket disconnected unexpectedly
                    if not self.ws_connected and self.ws is None and self.loop is None:
                        self.get_logger().info("WebSocket disconnected, returning to wake word mode")
                        self.mode = WAKE_WORD_MODE
                        decoder.end_utt()
                        decoder.start_utt()


def make_decoder(wake_word: str) -> Decoder:
    """Create a PocketSphinx decoder configured for wake word detection."""
    model_path = os.path.join(os.path.dirname(pocketsphinx.__file__), 'model')

    config = Decoder.default_config()
    config.set_string('-hmm', os.path.join(model_path, 'en-us', 'en-us'))
    config.set_string('-dict', os.path.join(model_path, 'en-us', 'cmudict-en-us.dict'))
    config.set_string('-logfn', '/dev/null')  # Suppress verbose pocketsphinx logs
    decoder = Decoder(config)

    # Configure wake word keyphrase spotting
    decoder.set_keyphrase(WAKE_WORD_MODE, wake_word)
    decoder.set_search(WAKE_WORD_MODE)


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