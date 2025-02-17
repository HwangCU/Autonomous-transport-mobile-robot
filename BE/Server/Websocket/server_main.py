# server_main.py

import os
import sys
import asyncio
import json
import ssl
from datetime import datetime, timedelta
import jwt
import base64
from asgiref.sync import sync_to_async
from robot_info import robot_connections, robot_locks, robot_ip_list
from client_info import client_connections, client_locks, client_ip_list

from vars import SERVER_ADDR, SERVER_PORT


# Django 환경 설정
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../c103")))
os.environ.setdefault("DJANGO_SETTINGS_MODULE", "c103.settings")
import django

django.setup()

from users.models import User
from robots.models import Robot


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
    """User ID로 사용자 검색"""
    try:
        return User.objects.get(id=user_id)
    except User.DoesNotExist:
        return None


@sync_to_async
def get_robot_by_id(robot_id):
    """Robot ID로 사용자 검색"""
    try:
        return Robot.objects.get(robot_id=robot_id)
    except Robot.DoesNotExist:
        return None


# websockets
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../")))

from websockets.asyncio.server import serve
from Websocket.server_login import handle_login, handle_social_login, handle_robot_login
from Websocket.server_register import handle_registration
from Websocket.server_mode_change import handle_mode_change  # 추가
from Websocket.server_robot import (
    handle_robot_registration,
    handle_request_robot_location,
)
from Websocket.server_address import handle_address_update
from Websocket.server_footpath import handle_footpath
from Websocket.server_profile import handle_get_profile, handle_edit_profile

ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
# ssl_context.load_cert_chain(
#     certfile="../Websocket/cert.pem", keyfile="../Websocket/key.pem"
# )
ssl_context.load_cert_chain(
    certfile="./fullchain.pem",
    keyfile="./privkey.pem",
)

# JWT 비밀키 로드


def load_jwt_key(file_path: str) -> bytes:
    with open(file_path, "r") as f:
        base64_key = f.read().strip()
    secret_bytes = base64.b64decode(base64_key)
    return secret_bytes


SECRET_KEY = load_jwt_key("./jwt_key.pem")


async def verify_access_token(token, expected_requester_type):
    """JWT Access Token 검증"""
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=["HS256"])  # JWT 디코딩
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

        return requester  # 정상적인 경우, User 객체 반환

    except jwt.ExpiredSignatureError:
        return {"status": "error", "message": "Access token has expired"}
    except jwt.InvalidTokenError:
        return {"status": "error", "message": "Invalid access token"}
    except Exception as e:
        return {"status": "error", "message": f"Token verification failed: {e}"}


async def handler(websocket):
    while True:
        try:
            remote_addr = websocket.remote_address  # (ip, port) 형태
            print(f"[handler] New connection from {remote_addr}")
            message = await websocket.recv()
            data = json.loads(message)
            print(data)

            action = data.get("action")
            print(f"\n{action}\n")

            if action in [
                "mode_change",
                "protected_action",
                "regist_robot",
                "address_regist",
                "request_location",
                "get_profile",
                "edit_profile",
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
                        print("success and user_id is ", user.id)
                        payload = {
                            "action": "mode_change",
                            "mode": data.get("mode"),
                        }
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

                    print(f"\n\nipaaddris {client_ip_list[user_id]}\n\n")
                    # 해당 로봇에 대한 Lock도 생성(없으면 새로 생성)
                    if user_id not in client_locks:
                        client_locks[user_id] = asyncio.Lock()

            elif action == "sns_login":
                response = await handle_social_login(data)
                print(response)
                user_id = await get_user_by_email(data)
                print(f"user_id id {user_id}")
                if user_id:
                    client_connections[user_id] = websocket
                    client_ip_list[user_id], _ = client_connections[
                        user_id
                    ].remote_address

                    print(f"\n\nipaaddris {client_ip_list[user_id]}\n\n")
                    # 해당 로봇에 대한 Lock도 생성(없으면 새로 생성)
                    if user_id not in client_locks:
                        client_locks[user_id] = asyncio.Lock()
            elif action == "robot_login":
                response = await handle_robot_login(data)
                # 로봇이 자신의 연결을 등록하는 메시지 (예: {"action": "robot_connect", "robot_id": robot_id})
                user_id = await get_user_by_robot_id(data)
                print(f"robot's user_id id {user_id}")
                if user_id:
                    robot_connections[user_id] = websocket
                    robot_ip_list[user_id], _ = robot_connections[
                        user_id
                    ].remote_address

                    print(f"\n\nipaaddris {robot_ip_list[user_id]}\n\n")
                    # 해당 로봇에 대한 Lock도 생성(없으면 새로 생성)
                    if user_id not in robot_locks:
                        robot_locks[user_id] = asyncio.Lock()
                else:
                    response = {"status": "error", "message": "robot_id is required"}

                print(
                    f"robot user ({user_id}) connection user id is !!!!!!!!!!1 {robot_connections[user_id].remote_address} !!!!!!!!!1"
                )
                await websocket.send(json.dumps(response))

            else:
                response = {"status": "error", "message": "Invalid action"}

            await websocket.send(json.dumps(response))

        except Exception as e:
            await websocket.send(
                json.dumps({"status": "error", "message": f"Server error: {e}"})
            )


async def main():
    async with serve(
        handler,
        SERVER_ADDR,
        SERVER_PORT,
        ssl=ssl_context,
        ping_interval=None,
        ping_timeout=None,
    ):
        print("Secure WebSocket server running on wss://0.0.0.0:8601")
        await asyncio.Future()


if __name__ == "__main__":
    asyncio.run(main())
