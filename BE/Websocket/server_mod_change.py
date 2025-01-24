# server_mod_change.py

from asgiref.sync import sync_to_async
from users.models import User


@sync_to_async
def get_user_by_id(user_id):
    """User ID로 사용자 검색"""
    try:
        return User.objects.get(id=user_id)
    except User.DoesNotExist:
        return None


async def handle_mod_change(data, user):
    """mod_change 처리"""
    try:
        new_mode = data.get("mode")

        if not new_mode:
            return {
                "status": "error",
                "message": "New mode is required",
            }

        # 모드 전환 로직 (예: 데이터베이스 업데이트)
        result = await change_mode_for_user(user, new_mode)

        if result:
            return {
                "status": "success",
                "message": f"Mode successfully changed to {new_mode}",
            }
        else:
            return {
                "status": "error",
                "message": "Failed to change mode",
            }

    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}


@sync_to_async
def change_mode_for_user(user, new_mode):
    """사용자 모드 변경 로직"""
    try:
        user.mode = new_mode  # 'mode' 필드가 있다고 가정
        user.save()
        return True
    except Exception as e:
        print(f"Error changing mode: {e}")
        return False
