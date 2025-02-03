# server_main.py

import os
import sys
import asyncio
import json
import ssl
from datetime import datetime, timedelta
import jwt
import base64

# Django 환경 설정
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../c103")))
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "c103.settings")
import django

django.setup()

# websockets
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))

from websockets.asyncio.server import serve
from Websocket.server_login import handle_login
from Websocket.server_register import handle_registration
from Websocket.server_mod_change import handle_mod_change, get_user_by_id  # 추가

ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.load_cert_chain(
    certfile="../Websocket/cert.pem", keyfile="../Websocket/key.pem"
)

# JWT 비밀키 로드


def load_jwt_key(file_path: str) -> bytes:
    with open(file_path, "r") as f:
        base64_key = f.read().strip()
    secret_bytes = base64.b64decode(base64_key)
    return secret_bytes


SECRET_KEY = load_jwt_key("./jwt_key.pem")


async def verify_access_token(token):
    """Access Token 검증"""
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=["HS256"])
        user_id = payload.get("user_id")
        if not user_id:
            return None

        user = await get_user_by_id(user_id)
        return user
    except jwt.ExpiredSignatureError:
        return {"status": "error", "message": "Access token has expired"}
    except jwt.InvalidTokenError:
        return {"status": "error", "message": "Invalid access token"}


async def handler(websocket):
    while True:
        try:
            message = await websocket.recv()
            data = json.loads(message)

            action = data.get("action")

            if action in ["mod_change", "protected_action"]:
                token = data.get("access_token")
                if not token:
                    response = {
                        "status": "error",
                        "message": "Access token is required",
                    }
                    await websocket.send(json.dumps(response))
                    continue

                user = await verify_access_token(token)
                if not user or isinstance(user, dict):  # 검증 실패
                    await websocket.send(json.dumps(user))
                    continue

                if action == "mod_change":
                    response = await handle_mod_change(data, user)
            elif action == "register":
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
