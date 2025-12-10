import os
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
SAMPLING_RATE = 44100


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

        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("STTOnboard listener started")

    def _listen_loop(
            self,
            buffer_size: int = 4096,
            channels: int = 1,
            wake_word: str = "echo listen",
            time_to_wait_in_silence_seconds: float = 2.0,
            max_listen_time_seconds: float = 15.0,
    ):
        """
        Uses PocketSphinx's built-in keyword spotting to listen for a wake word.
        Once detected, starts recording audio until silence and saves it to a .wav file.
        """
        audio_q = LifoQueue()
        frames = []

        def audio_callback(indata, frames, time, status):
            audio_q.put(indata.copy())

        self.get_logger().info(f"Listening for wake word '{wake_word}'...")

        decoder = make_decoder(wake_word)

        with sounddevice.InputStream(
                samplerate=SAMPLING_RATE,
                channels=channels,
                dtype='int16',
                blocksize=buffer_size,
                callback=audio_callback,
        ):
            decoder.start_utt()
            mode = WAKE_WORD_MODE

            while True:
                raw = audio_q.get().tobytes()
                decoder.process_raw(raw)

                hyp = decoder.hyp()

                if mode == WAKE_WORD_MODE and hyp:
                    self.get_logger().info(f"Wake word ({hyp.hypstr}) detected!")
                    mode = LANGUAGE_SEARCH_MODE
                    decoder.end_utt()
                    decoder.set_search(LANGUAGE_SEARCH_MODE)
                    decoder.start_utt()
                    frames = []
                    speech, speech_updated, start_time = None, time(), time()
                    self.sounds_pub.publish(String(data=BELL_SOUND))
                    continue
                elif mode == WAKE_WORD_MODE:
                    continue
                elif mode == LANGUAGE_SEARCH_MODE:
                    frames.append(raw)

                    if not hyp:
                        continue  # no speech yet

                    new_text = hyp.hypstr.lower()
                    if speech != new_text:
                        speech, speech_updated = new_text, time()
                        continue

                    if (
                            (time() - speech_updated > time_to_wait_in_silence_seconds)
                            or (time() - start_time > max_listen_time_seconds)
                    ):
                        # Consider utterance complete after a period of no change
                        decoder.end_utt()
                        self.sounds_pub.publish(String(data=BELL_END_SOUND))
                        wav_file = BytesIO()
                        with wave.open(wav_file, "wb") as wf:
                            wf.setnchannels(channels)
                            wf.setsampwidth(2)
                            wf.setframerate(SAMPLING_RATE)
                            wf.writeframes(b"".join(frames))
                        wav_file.seek(0)
                        self.ai_pub.publish(String(data=b64encode(wav_file.read()).decode()))

                        self.get_logger().info(f"Phrase is complete. Switching back to wake word detection.")
                        # Reset for wake word detection
                        decoder.set_search(WAKE_WORD_MODE)
                        decoder.start_utt()
                        mode = WAKE_WORD_MODE
                        frames = []
                else:
                    raise RuntimeError("Unknown mode in STTOnboard listen loop")


def make_decoder(wake_word: str) -> Decoder:
    model_path = os.path.join(os.path.dirname(pocketsphinx.__file__), 'model', 'en-us')

    config = Decoder.default_config()
    config.set_string('-hmm', os.path.join(model_path, 'en-us'))
    config.set_string('-dict', os.path.join(model_path, 'cmudict-en-us.dict'))
    config.set_string('-lm', os.path.join(model_path, 'en-us.lm.bin'))
    config.set_float('-samprate', SAMPLING_RATE)  # Set sampling rate
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