import threading
from queue import Queue
import wave

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import sounddevice

try:
    from pocketsphinx import Pocketsphinx
except Exception as e:
    raise RuntimeError(
        "Failed to import pocketsphinx. Make sure 'pocketsphinx' is installed.\n"
        "On Debian/Ubuntu: sudo apt install swig build-essential portaudio19-dev\n"
        "then pip install pocketsphinx pyaudio"
    ) from e


class STTOnboard(Node):
    """
    A simple ROS2 node that listens to the microphone via PocketSphinx LiveSpeech
    and publishes recognized phrases on /tts_onboard/say (std_msgs/String).
    """

    def __init__(self):
        super().__init__("stt_onboard")
        self.pub = self.create_publisher(String, "/tts_onboard/say", 10)

        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()
        self.get_logger().info("STTOnboard listener started")

    def _listen_loop(
            self,
            buffer_size: int = 1024,
            channels: int = 1,
            rate: int = 16000,
    ):
        """
        Note: pocketsphinx might register some false positives when starting up. This becomes less after some actual
        speech is detected.
        """
        pocketsphinx = Pocketsphinx()

        # Queue to collect audio data from the callback
        audio_q = Queue()

        def audio_callback(indata, frames, time, status):
            """Callback called from sounddevice every audio block."""
            audio_q.put(indata.copy())

        # Start PocketSphinx in utterance mode
        pocketsphinx.start_utt()
        frames = []
        phrase_index = 0

        self.get_logger().info("STT listening...")

        with sounddevice.InputStream(
                samplerate=rate,
                channels=channels,
                dtype='int16',
                blocksize=buffer_size,
                callback=audio_callback,
        ):
            while True:
                # Get audio chunk from queue
                data = audio_q.get()
                raw = data.tobytes()
                frames.append(raw)
                pocketsphinx.process_raw(raw, False, False)

                hyp = pocketsphinx.hyp()
                if hyp is not None:
                    self.get_logger().info(f"Recognized: {hyp.hypstr}")

                    # --- Save audio for this phrase ---
                    wav_filename = f"phrase_{phrase_index}.wav"
                    with wave.open(wav_filename, "wb") as wf:
                        wf.setnchannels(channels)
                        wf.setsampwidth(2)  # int16 -> 2 bytes
                        wf.setframerate(rate)
                        wf.writeframes(b"".join(frames))
                    self.get_logger().info(f"Saved audio to {wav_filename}\n")

                    # Reset for next utterance
                    frames = []
                    pocketsphinx.end_utt()
                    pocketsphinx.start_utt()
                    phrase_index += 1
        #
        # except KeyboardInterrupt:
        #     print("\nStopping...")
        # finally:
        #     pocketsphinx.end_utt()

        # The LiveSpeech iterator blocks and yields recognized chunks.
        # for phrase in self.speech:
        #     try:
        #         text = str(phrase).strip()
        #         if not text:
        #             continue
        #         msg = String()
        #         msg.data = text
        #         # publish on the ROS topic
        #         self.pub.publish(msg)
        #         self.get_logger().info(f"Recognized: {text}")
        #     except Exception as e:
        #         self.get_logger().error(f"Error during recognition loop: {e}")


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