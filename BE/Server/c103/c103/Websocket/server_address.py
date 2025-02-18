import json
from asgiref.sync import sync_to_async
from django.contrib.auth import get_user_model
from users.models import User


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


async def handle_address_update(data, user):
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
