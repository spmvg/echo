# Echo

Echo is an experimental open-source voice assistant built on ROS 2. It uses OpenAI's realtime voice API for low-latency conversational AI, with local wake-word detection to save on cloud costs. Designed to run on a Raspberry Pi.

Key features:
- Wake-word detection using `PocketSphinx` (no cloud costs until activated)
- Real-time voice conversation via OpenAI's WebSocket API
- Simple ROS 2 architecture

## Quickstart (Docker)

Build the development container:

```bash
docker build -t echo -f docker/Dockerfile .
```

Run the container with access to host sound devices (Linux):

```bash
export OPENAI_API_KEY=your_openai_key_here
docker run -it --rm \
  --env OPENAI_API_KEY=$OPENAI_API_KEY \
  --device /dev/snd \
  -v $(pwd)/ros2_workspace:/root/ros2_workspace \
  echo
```

Notes:
- `--device /dev/snd` is required for audio access on Linux.
- Replace `your_openai_key_here` with a valid OpenAI API key.

## Environment variables

- `OPENAI_API_KEY`: Your OpenAI API key for the realtime voice API.

## Setup on Raspberry Pi

1. Install Ubuntu Server on your Raspberry Pi.
2. Follow the commands in the [Dockerfile](docker/Dockerfile).
3. Configure audio input/output (e.g., USB microphone, 3.5mm audio jack). Example `/etc/asound.conf`:

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

4. Build and launch:

```bash
cd echo/ros2_workspace
source /opt/ros/kilted/setup.bash
colcon build --symlink-install
source install/local_setup.bash
ros2 launch echo all_nodes.launch.py
```

## How it works

1. Say "echo listen" to activate
2. Echo greets you and starts a conversation
3. Speak naturally — the assistant responds in real-time
4. After 30 seconds of inactivity, Echo returns to listening for the wake word

## Architecture

The ROS package `echo` contains:

- **`stt_onboard`** — Main node handling wake-word detection and OpenAI realtime voice communication
  - Uses PocketSphinx for local wake-word detection ("echo listen")
  - Streams audio to/from OpenAI's realtime WebSocket API
  - Handles audio input (microphone) and output (speaker) via `sounddevice`

- **`tts_onboard`** — Local text-to-speech using `pyttsx3` for startup announcements

- **`initialization`** — Startup node that checks connectivity and announces status

## Contributing

Contributions welcome. Please open issues for bugs and feature requests.

## License

This project is licensed under the AGPL-3.0 (see `LICENSE`).
