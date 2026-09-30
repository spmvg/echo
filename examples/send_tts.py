"""
Example: send a text-to-speech message to Echo via rosbridge WebSocket.

Usage:
    pip install websockets
    python send_tts.py "System message"
"""

import asyncio
import json
import sys

import websockets

PI_IP = "localhost"


async def main(text: str):
    async with websockets.connect(f"ws://{PI_IP}:9090") as ws:
        await ws.send(json.dumps({
            "op": "publish",
            "topic": "/tts_onboard/say",
            "msg": {"data": text},
        }))
        print(f"Sent TTS message: {text}")


if len(sys.argv) != 2:
    raise SystemExit(f"Usage: {sys.argv[0]} \"message\"")

asyncio.run(main(sys.argv[1]))