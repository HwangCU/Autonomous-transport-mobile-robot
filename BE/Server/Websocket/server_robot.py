import json
from asgiref.sync import sync_to_async
from robots.models import Robot


@sync_to_async
def create_robot(user, robot_id):
    # 이미 등록된 로봇이 있는지 확인할 수도 있음 (한 사용자당 하나로 제한할 경우)
    if Robot.objects.filter(user_id=user.id).exists():
        return None  # 이미 로봇이 등록된 경우
    robot = Robot.objects.create(user_id=user.id, robot_id=robot_id)
    return robot


async def handle_robot_registration(data, user):
    """
    WebSocket 메시지 예시:
    {
        "action": "register_robot",
        "robot_id": "my_robot_id"
    }
    """
    robot_id = data.get("robot_id")
    if not robot_id:
        return {"status": "error", "message": "Robot id is required"}

    robot = await create_robot(user, robot_id)
    if robot is None:
        return {"status": "error", "message": "Robot already registered"}
    return {
        "status": "success",
        "message": "Robot registered successfully",
    }
