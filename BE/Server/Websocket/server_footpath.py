# /Websocket/server_footpath.py

import json
from asgiref.sync import sync_to_async
from django.contrib.auth import get_user_model
from users.models import User

from robot_info import robot_connections, robot_locks


@sync_to_async
def update_user_address(user, address):
    """email을 가진 사용자의 주소를 업데이트"""
    try:
        user.address = address
        user.save()
        return {"status": "success", "message": "Address updated successfully"}
    except User.DoesNotExist:
        return {"status": "error", "message": "User not found"}
    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}


async def handle_footpath(data, user):
    """WebSocket으로 받은 데이터를 처리하여 사용자 주소 업데이트"""
    try:
        address = data.get("address")

        if not address:
            return {"status": "error", "message": "email and address are required"}

        # 사용자 주소 업데이트 실행
        response = await update_user_address(user, address)
        return response

    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}


async def handle_request_location(data, user):
    try:
        robot_id = data.get("robot_id")

        if robot_id in robot_connections:
            await robot_connections[robot_id].send(
                json.dumps({"action": "location_request"})
            )
            try:
                robot_response = await asyncio.wait_for(
                    robot_connections[robot_id].recv(), timeout=10
                )
                response = {
                    "status": "success",
                    "robot_location": json.loads(robot_response),
                }
            except asyncio.TimeoutError:
                response = {
                    "status": "error",
                    "message": "Robot did not respond in time",
                }

    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}

    """
    로봇의 위치를 요청하는 함수
    robot_ip: 로봇의 IP 주소 (예: "192.168.1.100")
    robot_port: 로봇의 포트 (예: 8000)
    """
    # TODO: 코드 가공해서 사용
    # 로봇의 위치 요청 URL (로봇이 자체적으로 운영하는 Django 앱의 엔드포인트)
    robot_url = f"http://{robot_ip}:{robot_port}/robot/location/"

    # 필요한 경우 추가 데이터(예: 인증 토큰, action 등)를 포함할 수 있음
    payload = {"action": "location_request"}

    try:
        response = requests.post(robot_url, json=payload, timeout=10)
        data = response.json()
        return data
    except requests.RequestException as e:
        return {"status": "error", "message": f"Request error: {e}"}

    """
    
            if action == "register_robot":
                robot_id = data.get("robot_id")
                if robot_id:
                    robot_connections[robot_id] = websocket
                    response = {"status": "success", "message": f"Robot {robot_id} registered"}
                else:
                    response = {"status": "error", "message": "robot_id is required"}
            elif action == "request_robot_location":
                robot_id = data.get("robot_id")
                if robot_id in robot_connections:
                    # 요청 전송
                    await robot_connections[robot_id].send(json.dumps({"action": "location_request"}))
                    try:
                        robot_response = await asyncio.wait_for(robot_connections[robot_id].recv(), timeout=10)
                        response = {"status": "success", "robot_location": json.loads(robot_response)}
                    except asyncio.TimeoutError:
                        response = {"status": "error", "message": "Robot did not respond in time"}
                else:
                    response = {"status": "error", "message": f"Robot {robot_id} not registered"}
            elif action == "location_response":
                # 로봇이 위치 응답을 보내면, 어떤 방식으로 앱에 전달할지 결정 (예: 별도 DB에 저장하거나 클라이언트와 연결된 WebSocket에 전달)
                # 이 예시에서는 단순히 로그로 출력
                print("Robot location response:", data)
                continue  # 즉시 처리
            # ... (다른 액션 처리)
            else:
                response = {"status": "error", "message": "Invalid action"}
    """
