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
| `LISTENING_DISABLED` | No | *(unset)* | Set to any non-empty value (e.g. `1`) to start with wake-word listening **off**. Echo will announce *"Listening disabled"* on startup and wait for a remote `/stt_onboard/set_listening` command to enable it. |
| `INACTIVITY_TIMEOUT` | No | `10` | Seconds of silence before a conversation is automatically closed |

## Remote control via rosbridge

Echo exposes its ROS 2 topics over a standard [rosbridge WebSocket](https://github.com/RobotWebTools/rosbridge_suite) server running on **port 9090**.
Any device that can reach the Pi over the network can publish and subscribe to topics using the [rosbridge protocol](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md) — a simple JSON-over-WebSocket API.
No extra broker or cloud service is needed.

### ROS 2 topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/stt_onboard/set_listening` | `std_msgs/Bool` | → Pi | `true` to enable wake word, `false` to disable |
| `/stt_onboard/listening_state` | `std_msgs/Bool` | ← Pi | Current listening state (latched — new subscribers get the latest value immediately) |

### Rosbridge WebSocket protocol

Connect to `ws://<PI_IP>:9090` and send/receive JSON frames.

**Publish a message** (set listening on or off):

```json
{
  "op": "publish",
  "topic": "/stt_onboard/set_listening",
  "msg": { "data": true }
}
```

**Subscribe to listening state** (receive updates whenever the state changes):

```json
{
  "op": "subscribe",
  "topic": "/stt_onboard/listening_state",
  "type": "std_msgs/Bool"
}
```

Each state update arrives as:

```json
{
  "op": "publish",
  "topic": "/stt_onboard/listening_state",
  "msg": { "data": true }
}
```


### Python example

```python
import asyncio, json
import websockets

PI_IP = "192.168.x.x"  # or Tailscale IP

async def main():
    async with websockets.connect(f"ws://{PI_IP}:9090") as ws:
        # Subscribe to listening state
        await ws.send(json.dumps({
            "op": "subscribe",
            "topic": "/stt_onboard/listening_state",
            "type": "std_msgs/Bool",
        }))
        state = await ws.recv()
        print("Current state:", json.loads(state))

        # Disable listening
        await ws.send(json.dumps({
            "op": "publish",
            "topic": "/stt_onboard/set_listening",
            "msg": {"data": False},
        }))

asyncio.run(main())
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
- **`rosbridge_websocket`** — Exposes all ROS 2 topics over WebSocket on port 9090 for remote control
- **`initialization`** — Startup checks and status announcements

## Contributing

Contributions welcome. Please open issues for bugs and feature requests.

## License

This project is licensed under the AGPL-3.0 (see `LICENSE`).
