# server_login.py

import jwt
from datetime import datetime, timedelta
from asgiref.sync import sync_to_async
from django.contrib.auth.hashers import check_password

from users.models import User

import base64


def load_jwt_key(file_path: str) -> bytes:
    with open(file_path, "r") as f:
        base64_key = f.read().strip()
    secret_bytes = base64.b64decode(base64_key)
    return secret_bytes


SECRET_KEY = load_jwt_key("./jwt_key.pem")  # 실제 경로 확인
ACCESS_TOKEN_EXPIRATION = 15  # 분
REFRESH_TOKEN_EXPIRATION = 7  # 일


@sync_to_async
def authenticate_user(username, password):
    try:
        user = User.objects.get(username=username)
        if check_password(password, user.password):
            return user
        return None
    except User.DoesNotExist:
        return None


def generate_tokens(user):
    now = datetime.utcnow()

    # Access Token
    access_token_payload = {
        "user_id": user.id,
        "username": user.username,
        "exp": now + timedelta(minutes=ACCESS_TOKEN_EXPIRATION),
    }
    access_token = jwt.encode(access_token_payload, SECRET_KEY, algorithm="HS256")

    # Refresh Token
    refresh_token_payload = {
        "user_id": user.id,
        "exp": now + timedelta(days=REFRESH_TOKEN_EXPIRATION),
    }
    refresh_token = jwt.encode(refresh_token_payload, SECRET_KEY, algorithm="HS256")

    return access_token, refresh_token


async def handle_login(data):
    try:
        username = data.get("username")
        password = data.get("password")

        if not username or not password:
            return {"status": "error", "message": "Username and password are required"}

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
