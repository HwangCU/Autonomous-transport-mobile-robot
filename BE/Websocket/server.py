import os
import sys
import asyncio
import json
import ssl
import base64
import jwt
from websockets.asyncio.server import serve
from django.core.exceptions import ValidationError
from asgiref.sync import sync_to_async
from datetime import datetime, timedelta
from django.contrib.auth.hashers import check_password
from django.contrib.auth.hashers import make_password


# Django 환경 설정
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../c103")))
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "c103.settings")

import django

django.setup()

from users.models import User


def load_jwt_key(file_path: str) -> bytes:
    """
    Base64로 인코딩된 키 파일(jwt_key.pem)을 읽어
    실제 랜덤 바이트(bytes)로 디코딩하여 반환.
    """
    with open(file_path, "r") as f:
        base64_key = f.read().strip()
    secret_bytes = base64.b64decode(base64_key)
    return secret_bytes


SECRET_KEY = load_jwt_key("./jwt_key.pem")
ACCESS_TOKEN_EXPIRATION = 15  # Access token 만료 시간 (분)
REFRESH_TOKEN_EXPIRATION = 7  # Refresh token 만료 시간 (일)

# TLS 설정
ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.load_cert_chain(
    certfile="../Websocket/cert.pem", keyfile="../Websocket/key.pem"
)


@sync_to_async
def authenticate_user(username, password):
    """사용자 인증"""
    try:
        user = User.objects.get(username=username)
        if check_password(password, user.password):
            return user
        return None
    except User.DoesNotExist:
        return None


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


def generate_tokens(user):
    """JWT 토큰 생성"""
    now = datetime.utcnow()

    # Access token
    access_token_payload = {
        "user_id": user.id,
        "username": user.username,
        "exp": now + timedelta(minutes=ACCESS_TOKEN_EXPIRATION),
    }
    access_token = jwt.encode(access_token_payload, SECRET_KEY, algorithm="HS256")

    # Refresh token
    refresh_token_payload = {
        "user_id": user.id,
        "exp": now + timedelta(days=REFRESH_TOKEN_EXPIRATION),
    }
    refresh_token = jwt.encode(refresh_token_payload, SECRET_KEY, algorithm="HS256")

    return access_token, refresh_token


async def handle_login(data):
    """로그인 처리"""
    try:
        username = data.get("username")
        password = data.get("password")

        # 필드 확인
        if not username or not password:
            return {"status": "error", "message": "Username and password are required"}

        # 사용자 인증
        user = await authenticate_user(username, password)
        if user:
            access_token, refresh_token = generate_tokens(user)
            return {
                "status": "success",
                "message": "Login successful",
                "access_token": access_token,
                "refresh_token": refresh_token,
            }
        else:
            return {"status": "error", "message": "Invalid username or password"}
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
            elif data.get("action") == "login":
                response = await handle_login(data)
            else:
                response = {"status": "error", "message": "Invalid action"}

            # 응답 전송
            await websocket.send(json.dumps(response))
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
