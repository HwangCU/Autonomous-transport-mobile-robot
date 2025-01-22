import os
import sys
import asyncio
import json
import ssl
from websockets.asyncio.server import serve
from django.core.exceptions import ValidationError
from asgiref.sync import sync_to_async  # 추가

# Django 환경 설정
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../c103")))
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "c103.settings")

import django

django.setup()

from users.models import User
from django.contrib.auth.hashers import make_password

# TLS 설정
ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.load_cert_chain(
    certfile="../Websocket/cert.pem", keyfile="../Websocket/key.pem"
)


@sync_to_async
def create_user(data):
    """사용자 생성 동작을 비동기로 처리."""
    user = User.objects.create(
        username=data["username"],
        password=make_password(data["password"]),  # bcrypt 해싱
        name=data["name"],
        email=data["email"],
    )
    user.save()


@sync_to_async
def check_user_exists(field, value):
    """특정 필드로 사용자 존재 여부 확인."""
    return User.objects.filter(**{field: value}).exists()


async def handle_registration(data):
    try:
        # 필수 필드 확인
        required_fields = {"username", "password", "name", "email"}
        if not required_fields.issubset(data.keys()):
            return {"status": "error", "message": "Missing required fields"}

        # 중복 확인
        if await check_user_exists("username", data["username"]):
            return {"status": "error", "message": "Username already exists"}
        if await check_user_exists("email", data["email"]):
            return {"status": "error", "message": "Email already exists"}

        # 사용자 생성
        await create_user(data)

        return {"status": "success", "message": "User registered successfully"}
    except ValidationError as e:
        return {"status": "error", "message": str(e)}
    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}


async def handler(websocket):
    while True:
        try:
            # 클라이언트로부터 메시지 수신
            message = await websocket.recv()
            data = json.loads(message)

            if data.get("action") == "register":
                response = await handle_registration(data)
                await websocket.send(json.dumps(response))
            else:
                await websocket.send(
                    json.dumps({"status": "error", "message": "Invalid action"})
                )
        except Exception as e:
            await websocket.send(
                json.dumps({"status": "error", "message": f"Server error: {e}"})
            )


async def main():
    async with serve(handler, "0.0.0.0", 8001, ssl=ssl_context):
        print("Secure WebSocket server running on wss://0.0.0.0:8001")
        await asyncio.Future()  # Run forever


if __name__ == "__main__":
    asyncio.run(main())
