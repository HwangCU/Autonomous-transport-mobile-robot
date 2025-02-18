import json
from asgiref.sync import sync_to_async
import asyncio
import requests

from robots.models import Robot
from users.models import User
from c103.Websocket.server_request import handle_request_robot

from c103.Websocket.robot_info import (
    robot_connections,
    robot_locks,
    robot_ip_list,
)  # 전역 변수 관리 모듈


@sync_to_async
def create_robot(user, robot_id):
    # 이미 등록된 로봇이 있는지 확인할 수도 있음 (한 사용자당 하나로 제한할 경우)
    if Robot.objects.filter(user_id=user).exists():
        return None  # 이미 로봇이 등록된 경우
    robot = Robot.objects.create(user_id=user, robot_id=robot_id)
    return robot


async def handle_robot_registration(data, user):
    """
    WebSocket 메시지 예시:
    {
        "action": "register_robot",
        "robot_id": "my_robot_id"
    }
    """
    print("robot registration user_id is ", user.id)
    robot_id = data.get("robot_id")
    if not robot_id:
        return {"status": "error", "message": "Robot id is required"}

    robot = await create_robot(user.id, robot_id)
    if robot is None:
        return {"status": "error", "message": "Robot already registered"}
    return {
        "status": "success",
        "message": "Robot registered successfully",
    }


@sync_to_async
def handle_request_robot_location(data, user, timeout=10):
    if user.id not in robot_connections:
        return {"status": "error", "message": f"Robot {user.id} not connected"}

    # 로봇 연결에 대한 Lock을 사용하여 동시에 여러 recv() 호출을 방지
    if user.id not in robot_locks:
        robot_locks[user.id] = asyncio.Lock()

    # 0218 TODO: 로봇으로는 POST를 보낼 수 없음. (싸피 또는 어떠한 공유기가 막고있음). 따라서 websocket을 활용한 요청이 필요함
    # 이 과정을 websocket으로 변경해서 전송할 필요 있음.
    # robot_url = f"http://{robot_ip}:9000/robot/location/"

    # 필요한 경우 추가 데이터(예: 인증 토큰, action 등)를 포함할 수 있음
    payload = {"action": "location_request"}

    response = handle_request_robot(payload, user)
    return response.json
