import json
from asgiref.sync import sync_to_async
import asyncio
import requests

from robots.models import Robot
from users.models import User

from robot_info import (
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
    """
    robot_id를 사용하여 해당 로봇에 위치 요청 메시지를 보내고, 응답을 기다립니다.
    """

    print("\nhandle_request_robot_location\n")
    robot = Robot.objects.get(user_id=user.id)
    print(robot)
    robot_id = robot.robot_id
    print(robot_id)

    if robot_id not in robot_connections:
        return {"status": "error", "message": f"Robot {robot_id} not connected"}

    robot_ip = robot_ip_list[robot_id]
    print(robot_ip)

    # 로봇 연결에 대한 Lock을 사용하여 동시에 여러 recv() 호출을 방지
    if robot_id not in robot_locks:
        robot_locks[robot_id] = asyncio.Lock()

    # TODO: 코드 가공해서 사용
    # 로봇의 위치 요청 URL (로봇이 자체적으로 운영하는 Django 앱의 엔드포인트)
    robot_url = f"http://{robot_ip}:9000/robot/location/"

    # 필요한 경우 추가 데이터(예: 인증 토큰, action 등)를 포함할 수 있음
    payload = {"action": "location_request"}

    try:
        response = requests.post(robot_url, json=payload, timeout=10)
        data = response.json()
        return data
    except requests.RequestException as e:
        return {"status": "error", "message": f"Request error: {e}"}
