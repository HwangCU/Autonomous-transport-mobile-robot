# 예시: login/server_login.py 내 handle_login 함수 수정

import jwt
from datetime import datetime, timedelta
from asgiref.sync import sync_to_async
from django.contrib.auth.hashers import check_password
from users.models import User, KakaoUser, GoogleUser
from robots.models import Robot  # 로봇 모델 임포트
from google.oauth2 import id_token
from google.auth.transport import requests

import base64
import requests


# JWT 관련 상수들
def load_jwt_key(file_path: str) -> bytes:
    with open(file_path, "r") as f:
        base64_key = f.read().strip()
    secret_bytes = base64.b64decode(base64_key)
    return secret_bytes


SECRET_KEY = load_jwt_key("./jwt_key.pem")
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

    access_token_payload = {
        "user_id": user.id,
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


async def handle_login(data):
    try:
        username = data.get("username")
        password = data.get("password")

        if not username or not password:
            return {"status": "error", "message": "Username and password are required"}

        user = await authenticate_user(username, password)
        if user:
            access_token, refresh_token = generate_tokens(user)

            # 로봇 등록 여부 확인
            robot_exists = await check_robot_registered(user)
            print(robot_exists)
            if not robot_exists:
                return {
                    "status": "success",
                    "username": username,
                    "message": "Login successful. Please register your robot.",
                    "require_robot": True,
                    "access_token": access_token,
                    "refresh_token": refresh_token,
                }
            else:
                return {
                    "status": "success",
                    "username": username,
                    "message": "Login successful.",
                    "require_robot": False,
                    "access_token": access_token,
                    "refresh_token": refresh_token,
                }
        else:
            return {"status": "error", "message": "Invalid username or password"}

    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}


@sync_to_async
def handle_social_login(data):
    """카카오 & 구글 로그인 처리"""
    print(data)
    try:
        userloginresource = data.get("userloginresource")  # "kakao" 또는 "google"
        token = data.get("token")  # 클라이언트에서 전달받은 access_token 또는 id_token

        if userloginresource == "kakao":
            verification = verify_kakao_access_token(token)
        elif userloginresource == "google":
            verification = verify_google_id_token(token)
        else:
            return {"status": "error", "message": "Invalid provider"}

        if verification["status"] == "success":
            user_info = verification["user_info"]
            email = user_info.get("kakao_account", {}).get("email")
            username = (
                user_info.get("kakao_account", {}).get("profile", {}).get("nickname")
            )
            usernum = user_info.get("id")
            key = user_info.get("key")
            if userloginresource == "kakao":
                user, created = User.objects.get_or_create(
                    username=username,
                    defaults={
                        "loginsource": "kakao",
                        "email": email,
                        "is_active": True,
                    },
                )
                kakao_user, _ = KakaoUser.objects.get_or_create(
                    user=user, usernum=usernum
                )
            elif userloginresource == "google":
                key = user_info.get("key")
                user, created = User.objects.get_or_create(
                    username=username,
                    defaults={
                        "loginsource": "google",
                        "email": email,
                        "is_active": True,
                    },
                )
                google_user, _ = GoogleUser.objects.get_or_create(
                    user=user, usernum=usernum, key=key
                )

            access_token, refresh_token = generate_tokens(user)

            return {
                "status": "success",
                "username": username,
                "usernum": usernum,
                "message": "Login successful.",
                "require_robot": created,
                "access_token": access_token,
                "refresh_token": refresh_token,
            }

        return verification

    except Exception as e:
        return {"status": "error", "message": f"Server error: {e}"}


def verify_kakao_access_token(access_token):
    """카카오 Access Token 검증"""

    print("카카오 검증")
    url = "https://kapi.kakao.com/v2/user/me"
    headers = {"Authorization": f"Bearer {access_token}"}

    response = requests.get(url, headers=headers)
    if response.status_code == 200:
        print("카카카오 검증 성공")
        user_info = response.json()
        print(user_info)
        return {"status": "success", "user_info": user_info}
    else:
        print("카카오 검증 실패")
        return {"status": "error", "message": "Invalid Kakao access token"}


def verify_google_id_token(id_token_str):
    """구글 ID 토큰 검증"""
    try:
        # 구글의 공개 키를 사용하여 ID 토큰 검증
        user_info = id_token.verify_oauth2_token(
            id_token_str, requests.Request(), "YOUR_GOOGLE_CLIENT_ID"
        )

        if user_info["iss"] not in [
            "accounts.google.com",
            "https://accounts.google.com",
        ]:
            return {"status": "error", "message": "Invalid issuer"}

        return {"status": "success", "user_info": user_info}
    except ValueError:
        return {"status": "error", "message": "Invalid Google ID token"}
