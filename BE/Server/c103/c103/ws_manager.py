# c103/ws_manager.py

import os
import sys
import asyncio
import json
import ssl
import base64
import jwt
import django
import threading

from asgiref.sync import sync_to_async
from datetime import datetime, timedelta
from django.http import JsonResponse


# --------------------------------------------------------
# 1. Django 환경 설정
# --------------------------------------------------------
# "c103" 폴더가 Django 프로젝트 최상단이라고 가정
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
sys.path.append(BASE_DIR)
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "c103.settings")
django.setup()

# 2. Models / DB 접근
from users.models import User
from robots.models import Robot

from c103.Websocket.robot_info import robot_connections, robot_locks, robot_ip_list
from c103.Websocket.client_info import client_connections, client_locks, client_ip_list
from robot_sensor_info import robot_gps

# 3. Websocket 관련 import
from websockets.asyncio.server import serve

# 4. Websocket 핸들러들
#    (server_login.py, server_register.py 등 기존 모듈 임포트)
from c103.Websocket.server_login import (
    handle_login,
    handle_social_login,
    handle_robot_login,
)
from c103.Websocket.server_register import handle_registration
from c103.Websocket.server_mode_change import handle_mode_change
from c103.Websocket.server_robot import (
    handle_robot_registration,
    handle_request_robot_location,
)
from c103.Websocket.server_address import handle_address_update
from c103.Websocket.server_footpath import handle_footpath, handle_send_footpath

from c103.Websocket.server_profile import handle_get_profile, handle_edit_profile


# --------------------------------------------------------
# 2) JWT Secret Key 로드
# --------------------------------------------------------
def load_jwt_key(file_path: str) -> bytes:
    with open(file_path, "r") as f:
        base64_key = f.read().strip()
    secret_bytes = base64.b64decode(base64_key)
    return secret_bytes


SECRET_KEY = load_jwt_key("./jwt_key.pem")


# --------------------------------------------------------
# 3) DB 접근 비동기 함수 (sync_to_async)
# --------------------------------------------------------
@sync_to_async
def get_user_by_email(data):
    try:
        email = data.get("email")
        user = User.objects.get(email=email)
        if user:
            return user.id
        return None
    except User.DoesNotExist:
        return None


@sync_to_async
def get_user_by_robot_id(data):
    try:
        robot_id = data.get("robot_id")
        robot = Robot.objects.get(robot_id=robot_id)
        if robot:
            return robot.user_id
        return None
    except Robot.DoesNotExist:
        return None


@sync_to_async
def get_user_by_id(user_id):
    try:
        return User.objects.get(id=user_id)
    except User.DoesNotExist:
        return None


@sync_to_async
def get_robot_by_id(robot_id):
    try:
        return Robot.objects.get(robot_id=robot_id)
    except Robot.DoesNotExist:
        return None


# --------------------------------------------------------
# 4) JWT 검증 함수
# --------------------------------------------------------
async def verify_access_token(token, expected_requester_type):
    """JWT Access Token 검증"""
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=["HS256"])
        print("\n\n 페이로드 \n\n", payload, "\n\n")

        requester_type = payload.get("requester_type")
        requester_id = payload.get(f"{requester_type}_id")

        if requester_type != expected_requester_type:
            return {"status": "error", "message": "Mismatched requester type"}

        if not requester_id:
            return {"status": "error", "message": "Invalid token structure"}

        if requester_type == "user":
            requester = await get_user_by_id(requester_id)
        elif requester_type == "robot":
            requester = await get_robot_by_id(requester_id)

        if requester is None:
            return {"status": "error", "message": "User not found"}

        return requester  # 정상
    except jwt.ExpiredSignatureError:
        return {"status": "error", "message": "Access token has expired"}
    except jwt.InvalidTokenError:
        return {"status": "error", "message": "Invalid access token"}
    except Exception as e:
        return {"status": "error", "message": f"Token verification failed: {e}"}


main_loop = None  # 웹소켓 서버 이벤트 루프를 담아둘 예정


async def send_to_client(user_id, payload):
    """
    Django(동기) 뷰에서 이 함수를 호출해,
    user_id에게 WebSocket 메시지를 보낼 수 있게 함.
    """
    print(client_connections)
    if user_id not in client_connections:
        print(f"[send_to_client] user_id={user_id} not connected.")
        return

    print(client_connections[user_id])

    ws = client_connections[user_id]

    print("\n\nready to send\n\n")

    try:
        await ws.send(json.dumps(payload))
        return JsonResponse(
            {"message": f"[send_to_client] Sent to user {user_id}: {payload}"}
        )
    except Exception as e:
        return JsonResponse({"message": f"[send_to_client] Error: {e}"})


# --------------------------------------------------------
# 5) WebSocket 핸들러
# --------------------------------------------------------
async def handler(websocket):
    remote_addr = websocket.remote_address  # (ip, port)
    print(f"[handler] New connection from {remote_addr}")

    while True:
        try:
            message = await websocket.recv()
            data = json.loads(message)
            print("[Received]", data)

            action = data.get("action", "")
            print(f"\nAction: {action}\n")

            # 토큰이 필요한 경우
            if action in [
                "mode_change",
                "protected_action",
                "regist_robot",
                "address_regist",
                "request_location",
                "get_profile",
                "edit_profile",
                "send_footpath",
            ]:
                token = data.get("access_token")
                if not token:
                    response = {
                        "status": "error",
                        "message": "Access token is required",
                    }
                    await websocket.send(json.dumps(response))
                    continue

                user = await verify_access_token(token, "user")
                if not user or isinstance(user, dict):  # 검증 실패
                    await websocket.send(json.dumps(user))
                    continue

                if action == "mode_change":
                    response = await handle_mode_change(data, user)
                    if response.get("status") == "success":
                        print("success -> user_id is", user.id)
                        payload = {
                            "action": "mode_change",
                            "mode": data.get("mode"),
                        }
                        # 로봇에게 전송
                        await robot_connections[user.id].send(json.dumps(payload))

                elif action == "regist_robot":
                    response = await handle_robot_registration(data, user)
                elif action == "address_regist":
                    response = await handle_address_update(data, user)
                elif action == "footpath":
                    response = await handle_footpath(data, user)
                elif action == "request_location":
                    response = await handle_request_robot_location(data, user)
                elif action == "get_profile":
                    response = await handle_get_profile(data, user)
                elif action == "edit_profile":
                    response = await handle_edit_profile(data, user)
                elif action == "send_footpath":
                    response = await handle_send_footpath(data, user)

            elif action == "register":
                response = await handle_registration(data)

            elif action == "login":
                response = await handle_login(data)
                user_id = await get_user_by_email(data)
                if user_id:
                    client_connections[user_id] = websocket
                    client_ip_list[user_id], _ = client_connections[
                        user_id
                    ].remote_address
                    print("연결된 클라이언트 확인: ", client_connections)
                    if user_id not in client_locks:
                        client_locks[user_id] = asyncio.Lock()

            elif action == "sns_login":
                response = await handle_social_login(data)
                user_id = await get_user_by_email(data)
                if user_id:
                    client_connections[user_id] = websocket
                    client_ip_list[user_id], _ = client_connections[
                        user_id
                    ].remote_address
                    if user_id not in client_locks:
                        client_locks[user_id] = asyncio.Lock()

            elif action == "robot_login":
                response = await handle_robot_login(data)
                user_id = await get_user_by_robot_id(data)
                if user_id:
                    robot_connections[user_id] = websocket
                    robot_ip_list[user_id], _ = robot_connections[
                        user_id
                    ].remote_address
                    if user_id not in robot_locks:
                        robot_locks[user_id] = asyncio.Lock()
                else:
                    response = {"status": "error", "message": "robot_id is required"}

            else:
                response = {"status": "error", "message": "Invalid action"}

            await websocket.send(json.dumps(response))

        except Exception as e:
            # 예외 처리
            error_msg = f"Server error: {e}"
            print(error_msg)
            await websocket.send(json.dumps({"status": "error", "message": error_msg}))
            break


# --------------------------------------------------------
# 6) WebSocket 서버 (asyncio)
# --------------------------------------------------------
# SSL 설정
ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
ssl_context.load_cert_chain(certfile="./fullchain.pem", keyfile="./privkey.pem")

HOST = "0.0.0.0"
PORT = 8501  # vars.SERVER_PORT (ex: 8501)


async def main():
    async with serve(
        handler,
        host=HOST,
        port=PORT,
        ssl=ssl_context,
        ping_interval=None,
        ping_timeout=None,
    ):
        print(f"Secure WebSocket server running on wss://{HOST}:{PORT}")
        await asyncio.Future()  # 무한 대기


# --------------------------------------------------------
# 7) 스레드를 통해 비동기 서버 구동
# --------------------------------------------------------
def start_websocket_server():
    """
    Django runserver와 동시에 WebSocket 서버를 구동하기 위해
    별도의 스레드에서 asyncio 이벤트 루프를 실행.
    """

    def run_in_thread():
        try:
            asyncio.run(main())
        except Exception as e:
            print("[WebSocketServer] Error in run_in_thread:", e)

    t = threading.Thread(target=run_in_thread, daemon=True)
    t.start()
    print("[WSManager] WebSocket server thread started.")
