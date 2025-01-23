# server_main.py

import os
import sys
import asyncio
import json
import ssl
import base64
from datetime import datetime, timedelta

# Django 환경 설정
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../c103")))
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "c103.settings")
import django

django.setup()

# websockets
from websockets.asyncio.server import serve

from server_login import handle_login
from BE.Websocket.ㅣㅁ시server_register import handle_registration

ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.load_cert_chain(
    certfile="../Websocket/cert.pem", keyfile="../Websocket/key.pem"
)


async def handler(websocket):
    while True:
        try:
            message = await websocket.recv()
            data = json.loads(message)

            action = data.get("action")
            if action == "register":
                response = await handle_registration(data)
            elif action == "login":
                response = await handle_login(data)
            else:
                response = {"status": "error", "message": "Invalid action"}

            await websocket.send(json.dumps(response))

        except Exception as e:
            await websocket.send(
                json.dumps({"status": "error", "message": f"Server error: {e}"})
            )


async def main():
    async with serve(handler, "0.0.0.0", 8001, ssl=ssl_context):
        print("Secure WebSocket server running on wss://0.0.0.0:8001")
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
