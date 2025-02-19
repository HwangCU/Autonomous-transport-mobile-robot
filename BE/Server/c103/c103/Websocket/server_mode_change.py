# server_mod_change.py

from asgiref.sync import sync_to_async
from users.models import User
from robots.models import Robot

from c103.Websocket.robot_info import robot_connections, robot_locks, robot_ip_list
from c103.Websocket.client_info import client_connections, client_locks, client_ip_list


@sync_to_async
def get_user_by_id(user_id):
    """User ID로 사용자 검색"""
    try:
        return User.objects.get(id=user_id)
    except User.DoesNotExist:
        return None


async def handle_mode_change(data, user):
    """mod_change 처리"""
    try:
        new_mode = data.get("mode")

        if new_mode == None:
            return {
                "status": "error",
                "message": "New mode is required",
            }

        # 모드 전환 로직 (예: 데이터베이스 업데이트)

        if True:
            print("mode change true")
            return {
                "status": "success",
                "message": f"Mode successfully changed to {new_mode}",
                "mode": new_mode,
            }
        else:
            return {
                "status": "error",
                "message": "Failed to change mode",
            }

    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}
