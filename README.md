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

| Variable | Required | Default | Description |
|---|---|---|---|
| `OPENAI_API_KEY` | **Yes** | — | OpenAI API key for the realtime voice API |
| `MODEL` | No | `gpt-realtime-mini` | OpenAI model to use |
| `PROMPT` | No | *(built-in)* | Custom personality prompt for the assistant |
| `MQTT_BROKER_HOST` | No | — | Hostname/IP of the MQTT broker (enables remote control when set) |
| `MQTT_BROKER_PORT` | No | `1883` | MQTT broker port |
| `MQTT_PREFIX` | No | `echo` | Prefix for all MQTT topics |

## Remote control via MQTT (optional)

When `MQTT_BROKER_HOST` is set, the `mqtt_bridge` node connects to the broker and exposes remote control over MQTT.
If the variable is not set, the bridge is disabled and Echo runs standalone — exactly as without MQTT.

This is designed for use over [Tailscale](https://tailscale.com): the Pi and the MQTT server join the same Tailnet, and you use the Tailscale IP as the broker host.

### Topics

| MQTT topic | Direction | Payload | Description |
|---|---|---|---|
| `{prefix}/listening/set` | **→ Pi** | `on` / `off` | Enable or disable wake word listening |
| `{prefix}/listening/state` | **← Pi** | `on` / `off` | Current listening state (retained) |

`{prefix}` defaults to `echo` (configurable via `MQTT_PREFIX`).

### Example

```bash
# Disable wake word listening
mosquitto_pub -h 100.x.x.x -t echo/listening/set -m off

# Enable wake word listening
mosquitto_pub -h 100.x.x.x -t echo/listening/set -m on

# Monitor state changes
mosquitto_sub -h 100.x.x.x -t echo/listening/state
```

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
4. After 10 seconds of inactivity, Echo returns to listening for the wake word

## Architecture

The ROS package `echo` contains:

- **`stt_onboard`** — Wake-word detection and OpenAI realtime voice communication
- **`tts_onboard`** — Local text-to-speech for status announcements
- **`mqtt_bridge`** — Optional MQTT ↔ ROS 2 bridge for remote control (disabled when `MQTT_BROKER_HOST` is not set)
- **`initialization`** — Startup checks and status announcements

## Contributing

Contributions welcome. Please open issues for bugs and feature requests.

## License

This project is licensed under the AGPL-3.0 (see `LICENSE`).
