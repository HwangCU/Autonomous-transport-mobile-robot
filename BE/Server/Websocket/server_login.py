# 예시: login/server_login.py 내 handle_login 함수 수정

import jwt
from datetime import datetime, timedelta
from asgiref.sync import sync_to_async
from django.contrib.auth.hashers import check_password
from users.models import User, LocalUser, KakaoUser, GoogleUser
from robots.models import Robot  # 로봇 모델 임포트
from google.auth.transport import requests as google_requests
from google.oauth2 import id_token
from SECRET import GOOGLE_CLIENT_ID

import base64
import requests
import asyncio


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
def authenticate_user(email, password):
    try:
        user = User.objects.get(email=email)
        user_pass = LocalUser.objects.get(user_id=user.id)
        if check_password(password, user_pass.password):
            return user.id
        return None
    except User.DoesNotExist:
        return None


def generate_tokens(user):
    now = datetime.utcnow()

    # if userloginresource=="local":
    #     usernum = LocalUser.objects.get(user_id=user.id).get("user_id")
    # elif userloginresource=="kakao":
    #     usernum = KakaoUser.objects.get(user_id=user.id).get("usernum")
    # elif userloginresource=="google":
    #     usernum = GoogleUser.objects.get(user_id=user.id).get("usernum")

    access_token_payload = {
        "user_id": user,
        "exp": now + timedelta(minutes=ACCESS_TOKEN_EXPIRATION),
    }
    access_token = jwt.encode(access_token_payload, SECRET_KEY, algorithm="HS256")

    refresh_token_payload = {
        "user_id": user,
        "exp": now + timedelta(days=REFRESH_TOKEN_EXPIRATION),
    }
    refresh_token = jwt.encode(refresh_token_payload, SECRET_KEY, algorithm="HS256")

    return access_token, refresh_token


@sync_to_async
def check_robot_registered(user_id):
    return not Robot.objects.filter(user_id=user_id).exists()


async def handle_login(data):
    try:
        email = data.get("email")
        password = data.get("password")

        if not email or not password:
            return {"status": "error", "message": "email and password are required"}

        user = await authenticate_user(email, password)
        if user:
            access_token, refresh_token = generate_tokens(user)

            # 로봇 등록 여부 확인
            robot_required = await check_robot_registered(user)
            print(robot_required)
            if robot_required:
                return {
                    "status": "success",
                    "email": email,
                    "message": "Login successful. Please register your robot.",
                    "require_robot": robot_required,
                    "access_token": access_token,
                    "refresh_token": refresh_token,
                }
            else:
                return {
                    "status": "success",
                    "email": email,
                    "message": "Login successful.",
                    "require_robot": robot_required,
                    "access_token": access_token,
                    "refresh_token": refresh_token,
                }
        else:
            return {"status": "error", "message": "Invalid email or password"}

    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}


@sync_to_async
def handle_social_login(data):
    """카카오 & 구글 로그인 처리"""
    try:
        userloginresource = data.get("userloginresource")  # "kakao" 또는 "google"
        token = data.get("token")  # 클라이언트에서 전달받은 access_token 또는 id_token

        if userloginresource == "kakao":
            verification = verify_kakao_access_token(token)
            email = verification.get("email")
            usernum = verification.get("id")
            user_id = User.objects.get(email=email).id
        elif userloginresource == "google":
            verification = verify_google_id_token(token)
            uinfo = verification.get("user_info")
            usernum = uinfo.get("sub")
            email = uinfo.get("email")
            user_id = User.objects.get(email=email).id

        else:
            return {"status": "error", "message": "Invalid provider"}

        print(f"verification is {verification}")

        if verification["status"] == "success":
            access_token, refresh_token = generate_tokens(user_id)

            return {
                "status": "success",
                "email": email,
                "usernum": usernum,
                "message": "Login successful.",
                "require_robot": verification.get("require_robot"),
                "access_token": access_token,
                "refresh_token": refresh_token,
            }

        return verification

    except Exception as e:
        return {"status": "error", "message": f"Server error: {e}"}


def verify_kakao_access_token(access_token):
    """카카오 Access Token 검증"""
    url = "https://kapi.kakao.com/v2/user/me"
    headers = {"Authorization": f"Bearer {access_token}"}

    response = requests.get(url, headers=headers)
    if response.status_code == 200:
        user_info = response.json()
        print(user_info)
        return regist_kakao(user_info)
    else:
        return {"status": "error", "message": "Invalid Kakao access token"}


def verify_google_id_token(id_token_str):
    """구글 ID 토큰 검증"""
    try:
        # ✅ google.auth.transport.requests.Request()를 사용해야 함
        request = google_requests.Request()

        # 구글의 공개 키를 사용하여 ID 토큰 검증
        user_info = id_token.verify_oauth2_token(
            id_token_str, request, GOOGLE_CLIENT_ID
        )

        # 발급자가 Google인지 확인
        if user_info["iss"] not in [
            "accounts.google.com",
            "https://accounts.google.com",
        ]:
            return {"status": "error", "message": "Invalid issuer"}

        return regist_google(user_info)

    except ValueError:
        return {"status": "error", "message": "Invalid Google ID token"}


def regist_kakao(user_info):
    email = user_info.get("kakao_account", {}).get("profile", {}).get("nickname")
    usernum = user_info.get("id")
    key = user_info.get("key")
    user, created = User.objects.get_or_create(
        email=email,
        defaults={
            "loginsource": "kakao",
            "email": email,
            "is_active": True,
        },
    )
    kakao_user, created = KakaoUser.objects.get_or_create(user=user, usernum=usernum)

    robot_exists = check_robot_registered(
        user.id
    )  # sync_to_async(check_robot_registered, thread_sensitive=True)(user.id)

    return {
        "status": "success",
        "user_info": user_info,
        "usernum": usernum,
        "email": email,
        "created": created,
        "require_robot": robot_exists,
    }


def regist_google(user_info):
    usernum = user_info.get("sub")
    email = user_info.get("email")
    user, created = User.objects.get_or_create(
        email=email,
        defaults={
            "loginsource": "google",
            "email": email,
            "is_active": True,
        },
    )
    google_user, created = GoogleUser.objects.get_or_create(user=user, usernum=usernum)

    robot_exists = check_robot_registered(
        user.id
    )  # sync_to_async(check_robot_registered, thread_sensitive=True)(user.id)

    return {
        "status": "success",
        "user_info": user_info,
        "usernum": usernum,
        "email": email,
        "created": created,
        "require_robot": robot_exists,
    }
