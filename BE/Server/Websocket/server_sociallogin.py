# 예시: login/server_login.py 내 handle_login 함수 수정

import jwt
from datetime import datetime, timedelta
from asgiref.sync import sync_to_async
from users.models import User
from robots.models import Robot  # 로봇 모델 임포트
import base64


# JWT 관련 상수들
def load_jwt_key(file_path: str) -> bytes:
    with open(file_path, "r") as f:
        base64_key = f.read().strip()
    secret_bytes = base64.b64decode(base64_key)
    return secret_bytes


SECRET_KEY = load_jwt_key("./jwt_key.pem")
ACCESS_TOKEN_EXPIRATION = 15  # 분
REFRESH_TOKEN_EXPIRATION = 7  # 일


def generate_tokens(user):
    now = datetime.utcnow()

    access_token_payload = {
        "user_id": user.id,
        "username": user.username,
        "exp": now + timedelta(minutes=ACCESS_TOKEN_EXPIRATION),
    }
    access_token = jwt.encode(access_token_payload, SECRET_KEY, algorithm="HS256")

    refresh_token_payload = {
        "user_id": user.id,
        "exp": now + timedelta(days=REFRESH_TOKEN_EXPIRATION),
    }
    refresh_token = jwt.encode(refresh_token_payload, SECRET_KEY, algorithm="HS256")

    return access_token, refresh_token


@sync_to_async
def check_robot_registered(user):
    return Robot.objects.filter(user=user).exists()


async def handle_sociallogin(data):
    try:
        username = data.get("username")
        address = data.get("address")
        social = data.get("social")

        # 추후 수정 필요
        user = username + address + social

        if not username or not address or not social:
            return {"status": "error", "message": "Username and password are required"}

        if user:
            access_token, refresh_token = generate_tokens(user)

            # 로봇 등록 여부 확인
            robot_exists = await check_robot_registered(user)
            print(robot_exists)
            if not robot_exists:
                return {
                    "status": "require_robot",
                    "message": "Login successful. Please register your robot.",
                    "require_robot": True,  # 클라이언트에게 로봇 등록 필요 신호
                    "access_token": access_token,
                    "refresh_token": refresh_token,
                }
            else:
                return {
                    "status": "success",
                    "message": "Login successful.",
                    "access_token": access_token,
                    "refresh_token": refresh_token,
                }
        else:
            return {"status": "error", "message": "Invalid username or password"}

    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}
