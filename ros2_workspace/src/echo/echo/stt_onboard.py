import asyncio
import base64
import json
import os
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray

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
MODEL = "gpt-4o-realtime-preview"
WS_URL = f"wss://api.openai.com/v1/realtime?model={MODEL}"
SAMPLE_RATE = 16000  # 16kHz to ease load on RPi
OPENAI_SAMPLE_RATE = 24000  # OpenAI realtime API uses 24kHz
CHANNELS = 1
DTYPE = "int16"
CHUNK_SIZE = 4096  # Larger chunks to reduce callback frequency on RPi
INACTIVITY_TIMEOUT = 20.0  # Seconds without model response before closing conversation


def pcm16_to_base64(audio: np.ndarray) -> str:
    return base64.b64encode(audio.tobytes()).decode("utf-8")

def base64_to_pcm16(data: str) -> np.ndarray:
    return np.frombuffer(base64.b64decode(data), dtype=DTYPE)

def resample_audio(audio: np.ndarray, from_rate: int, to_rate: int) -> np.ndarray:
    """Resample audio from one sample rate to another using linear interpolation."""
    if from_rate == to_rate:
        return audio
    duration = len(audio) / from_rate
    new_length = int(duration * to_rate)
    indices = np.linspace(0, len(audio) - 1, new_length)
    return np.interp(indices, np.arange(len(audio)), audio.astype(np.float32)).astype(DTYPE)


class STTOnboard(Node):
    """
    A ROS2 node that listens for a wake word via PocketSphinx, then streams
    audio to OpenAI's realtime voice API via WebSocket for conversation.
    """

    def __init__(self):
        super().__init__("stt_onboard")
        self.pub = self.create_publisher(String, "/tts_onboard/say", 10)
        self.ai_pub = self.create_publisher(String, "/speech_ai/audio", 10)
        # Subscribe to audio from tts_onboard for playback (avoids audio device conflicts)
        self.audio_sub = self.create_subscription(
            Int16MultiArray, "/stt_onboard/audio_out", self._on_tts_audio, 10
        )
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        if not self.openai_api_key:
            raise RuntimeError('No OPENAI_API_KEY, cannot proceed.')

        self.ws = None
        self.ws_connected = False
        self.mode = WAKE_WORD_MODE
        self.audio_out_buffer = np.array([], dtype=DTYPE)  # Continuous audio buffer
        self.audio_out_lock = threading.Lock()
        self.audio_in_queue = queue.Queue()
        self.lock = threading.Lock()
        self.loop = None
        self.last_activity = 0.0  # Timestamp of last activity (audio received from OpenAI)

        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("STTOnboard listener started")

    def _on_tts_audio(self, msg: Int16MultiArray):
        """Receive audio from tts_onboard and add to output buffer for playback."""
        audio = np.array(msg.data, dtype=DTYPE)
        with self.audio_out_lock:
            self.audio_out_buffer = np.append(self.audio_out_buffer, audio)
        self.get_logger().debug(f"Received {len(audio)} audio samples from tts_onboard")

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

        prompt = os.getenv("PROMPT") or f"Your personality is: helpful, creative, clever, and friendly."

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
                        "turn_detection": {
                            "type": "server_vad",
                            "threshold": 0.5,
                            "prefix_padding_ms": 300,
                            "silence_duration_ms": 500
                        },
                        "instructions": (
                            f"You are a concise voice assistant named Echo. {prompt}\n\n"
                            f"Your responses should be extremely short (1 sentence only and keep compound sentences to a minimum). Respond to the user input accordingly. You must be the first to say hello. Most importantly: be brief (1 sentence only) and concise."
                        )
                    }
                }
                await ws.send(json.dumps(session_update))

                # Trigger OpenAI to start the conversation (say hello first)
                await ws.send(json.dumps({
                    "type": "response.create",
                    "response": {
                        "modalities": ["audio", "text"]
                    }
                }))

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
            # Resample from OpenAI's 24kHz to our 16kHz
            audio = resample_audio(audio, OPENAI_SAMPLE_RATE, SAMPLE_RATE)
            # Append to buffer - use lock since speaker_callback runs in another thread
            with self.audio_out_lock:
                self.audio_out_buffer = np.append(self.audio_out_buffer, audio)
            self.last_activity = time.time()

        elif event_type == "response.audio.done":
            self.get_logger().debug("Audio response complete")
            self.last_activity = time.time()

        elif event_type == "response.done":
            self.get_logger().info("Response complete")
            self.last_activity = time.time()

        elif event_type == "error":
            self.get_logger().error(f"API error: {event.get('error', {})}")

        elif event_type == "session.created":
            self.get_logger().info("Session created")

        elif event_type == "session.updated":
            self.get_logger().info("Session updated")

    def send_audio_to_ws(self, audio_data: np.ndarray):
        """Send audio data to WebSocket if connected."""
        if self.loop and self.ws_connected:
            # Resample from our 16kHz to OpenAI's 24kHz
            audio_resampled = resample_audio(audio_data.flatten(), SAMPLE_RATE, OPENAI_SAMPLE_RATE)
            audio_b64 = pcm16_to_base64(audio_resampled)
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

        with self.audio_out_lock:
            if len(self.audio_out_buffer) >= frames:
                # Take exactly 'frames' samples from the buffer
                outdata[:] = self.audio_out_buffer[:frames].reshape(-1, 1)
                self.audio_out_buffer = self.audio_out_buffer[frames:]
            elif len(self.audio_out_buffer) > 0:
                # Partial buffer - play what we have, pad with silence
                available = len(self.audio_out_buffer)
                outdata[:available] = self.audio_out_buffer.reshape(-1, 1)
                outdata[available:] = 0
                self.audio_out_buffer = np.array([], dtype=DTYPE)
            else:
                # No audio available - output silence
                outdata.fill(0)

    def start_conversation(self):
        """Start a WebSocket connection for conversation."""
        self.last_activity = time.time()  # Reset activity timer

        def run_async_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self.ws_handler())
            self.loop.close()
            self.loop = None

        ws_thread = threading.Thread(target=run_async_loop, daemon=True)
        ws_thread.start()

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

        # Note: Don't clear audio buffer here - let any remaining audio finish playing
        # and allow the "Disconnected" TTS message to be queued

        # Notify user via TTS
        msg = String()
        msg.data = "Disconnected"
        self.pub.publish(msg)

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

                    # Check for inactivity timeout
                    if self.last_activity > 0 and (time.time() - self.last_activity) > INACTIVITY_TIMEOUT:
                        self.get_logger().info(f"No model response for {INACTIVITY_TIMEOUT}s, ending conversation")
                        self.end_conversation()
                        decoder.end_utt()
                        decoder.start_utt()
                        continue

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