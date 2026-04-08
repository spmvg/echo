"""
Example: control Echo's wake-word listening state via rosbridge WebSocket.

Usage:
    pip install websockets
    python set_listening.py
"""

import asyncio
import json

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
        print("Sent: disable listening")

        # Re-enable listening
        await ws.send(json.dumps({
            "op": "publish",
            "topic": "/stt_onboard/set_listening",
            "msg": {"data": True},
        }))
        print("Sent: enable listening")

        # Read updated state
        state = await ws.recv()
        print("Updated state:", json.loads(state))


asyncio.run(main())

