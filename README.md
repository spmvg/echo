# Echo

Echo is an experimental open-source interactive voice chat system built on ROS 2. It combines local onboard speech-to-text and text-to-speech with cloud-based AI (OpenAI) for transcription and response generation. The project is designed to run on a Raspberry Pi and can be packaged into a Docker container for easy development and testing.

Key features:
- Wake-word based local speech capture using `PocketSphinx` to save on cloud costs
- Local text-to-speech using `pyttsx3` to save on cloud costs
- Local sound playback for UI cues
- Cloud AI transcription and response (OpenAI Whisper + Responses API)
- Simple ROS 2 architecture so nodes can be run independently or via a single launch file

## Quickstart (Docker)

Build the development container (project root):

```bash
docker build -t echo -f docker/Dockerfile .
```

Run the container and give it access to the host sound devices (Linux):

```bash
export OPENAI_API_KEY=your_openai_key_here
docker run -it --rm \
  --env OPENAI_API_KEY=$OPENAI_API_KEY \
  --device /dev/snd \
  -v $(pwd)/ros2_workspace:/root/ros2_workspace \
  echo
```

Notes:
- `--device /dev/snd` is required to access audio on Linux hosts; it does not work on Windows.
- Replace `your_openai_key_here` with a valid OpenAI API key if you want cloud transcription + responses.

## Environment variables

- `OPENAI_API_KEY`: Your OpenAI API key for Whisper and Responses API.
- `PROMPT` (optional): Default system prompt used by `speech_ai` when querying the Responses API.

## Setup on Raspberry Pi
* Install Ubuntu Server on your Raspberry Pi.
* Follow the commands in the [Dockerfile](docker/Dockerfile).
* Configure audio input/output on your Raspberry Pi (e.g. USB microphone, 3.5mm audio jack).
    * It is recommended to write your own `/etc/asound.conf`/`~/.asoundrc` file to set the default input/output devices and use ALSA plug for sample rate conversion. For example:
```
pcm.usbplayback {
    type plug
    slave.pcm "dmix:UACDemoV10,0"
}

pcm.usbmic {
    type plug
    slave.pcm "dsnoop:Device,0"
}

pcm.!default {
    type asym
    playback.pcm "usbplayback"
    capture.pcm "usbmic"
}
```
* Go into the `echo/ros2_workspace` directory and launch all nodes:
```bash
source /opt/ros/kilted/setup.bash && colcon build --symlink-install && source install/local_setup.bash && ros2 launch echo all_nodes.launch.py
```

## Architecture & ROS components

The ROS package `echo` lives in `ros2_workspace/src/echo/echo/` and contains the following nodes and modules:

- `stt_onboard` (ROS node: `stt_onboard`)
  - Purpose: Local speech capture and wake-word detection using PocketSphinx and the system microphone.
  - Publishes: `/speech_ai/audio` (std_msgs/String) — base64-encoded WAV after a captured utterance
  - Publishes: `/tts_onboard/say` (std_msgs/String) — short status messages
  - Publishes: `/sounds/play` (std_msgs/String) — play short UI sounds (two built-in WAV files supplied)
  - Notes: Uses pocketsphinx keyword spotting to listen for the wake phrase (default: "echo listen"). Requires system audio input and pocketsphinx models available.

- `tts_onboard` (ROS node: `tts_onboard`)
  - Purpose: Local text-to-speech using pyttsx3.
  - Subscribes: `/tts_onboard/say` (std_msgs/String) — text to speak
  - Notes: Runs a background worker to avoid blocking ROS callbacks. Rate is configurable in code.

- `sound_player` (ROS node: `sound_player`)
  - Purpose: Plays short WAV UI sounds (bells) using sounddevice.
  - Subscribes: `/sounds/play` (std_msgs/String) — a string matching packaged WAV files
  - Notes: WAV files live under `ros2_workspace/src/echo/echo/sounds/` and are installed with the package.

- `speech_ai` (ROS node: `speech_ai`)
  - Purpose: Optional cloud processing — sends captured audio (base64 WAV) to OpenAI Whisper for transcription and to the Responses API for a short reply.
  - Subscribes: `/speech_ai/audio` (std_msgs/String) — base64-encoded WAV
  - Publishes: `/tts_onboard/say` (std_msgs/String) — response text for TTS
  - Notes: Requires `OPENAI_API_KEY` environment variable to be set for cloud capabilities. Uses the openai Python package.

- `initialization` (ROS node: `initialization`)
  - Purpose: Small helper node that runs at startup to check internet connectivity and environment variables, and announces status via `/tts_onboard/say`.
  - Publishes: `/tts_onboard/say` (std_msgs/String)

Launch orchestration
- A single launch file is provided at `ros2_workspace/src/echo/launch/all_nodes.launch.py` which starts all five nodes: `stt_onboard`, `tts_onboard`, `initialization`, `sound_player`, and `speech_ai`.

Topics summary
- `/tts_onboard/say` (std_msgs/String): Text-to-speech input and short status messages.
- `/sounds/play` (std_msgs/String): Request the local sound player to play a named WAV.
- `/speech_ai/audio` (std_msgs/String): Base64-encoded WAV data captured by `stt_onboard` for cloud transcription/AI.

## Contributing
Contributions welcome. Please open issues for bugs and feature requests. Follow the repository license and add tests for new behaviors.

## License
This project is licensed under the AGPL-3.0 (see `LICENSE`).
