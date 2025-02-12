# server_profile.py
from asgiref.sync import sync_to_async
from robots.models import Robot  # 로봇 정보를 저장하는 모델. robots 앱에 있다고 가정


@sync_to_async
def handle_get_profile(data, user):
    """
    사용자의 프로필 정보를 가져옵니다.

    반환 정보:
      - robot_id: 사용자의 로봇이 등록되어 있다면 해당 로봇의 robot_id, 없으면 None
      - address: 사용자의 주소
      - username: 사용자의 이름
      - email: 사용자의 이메일
    """
    try:
        # 해당 사용자의 로봇이 등록되어 있다면 첫 번째 로봇의 robot_id를 가져옴.
        robot = Robot.objects.filter(user=user).first()
        profile = {
            "robot_id": robot.robot_id if robot else None,
            "address": user.address,
            "username": user.username,
            "email": user.email,
        }
        return {"status": "success", "profile": profile}
    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}


@sync_to_async
def handle_edit_profile(data, user):
    """
    사용자의 프로필 정보를 수정합니다.

    수정 가능한 정보:
      - address
      - username

    수정 후 업데이트된 프로필 정보를 반환합니다.
    """
    try:
        new_address = data.get("address")
        new_username = data.get("username")

        if new_address is not None:
            user.address = new_address
        if new_username is not None:
            user.username = new_username

        user.save()

        robot = Robot.objects.filter(user=user).first()
        updated_profile = {
            "robot_id": robot.robot_id if robot else None,
            "address": user.address,
            "username": user.username,
            "email": user.email,
        }
        return {"status": "success", "profile": updated_profile}
    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}
