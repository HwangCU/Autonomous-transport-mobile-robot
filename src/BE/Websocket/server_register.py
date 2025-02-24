# server_register.py

from asgiref.sync import sync_to_async
from django.core.exceptions import ValidationError
from django.contrib.auth.hashers import make_password
from users.models import User


@sync_to_async
def create_user(data):
    """비동기로 사용자 생성"""
    user = User.objects.create(
        username=data["username"],
        password=make_password(data["password"]),  # bcrypt 해싱
        name=data["name"],
        email=data["email"],
    )
    user.save()


@sync_to_async
def check_user_exists(field, value):
    """필드 중복 확인"""
    return User.objects.filter(**{field: value}).exists()


async def handle_registration(data):
    """회원가입 처리"""
    try:
        required_fields = {"username", "password", "name", "email"}
        if not required_fields.issubset(data.keys()):
            return {"status": "error", "message": "Missing required fields"}

        # 중복 확인
        if await check_user_exists("username", data["username"]):
            return {"status": "error", "message": "Username already exists"}
        if await check_user_exists("email", data["email"]):
            return {"status": "error", "message": "Email already exists"}

        # 사용자 생성
        await create_user(data)
        return {"status": "success", "message": "User registered successfully"}

    except ValidationError as e:
        return {"status": "error", "message": str(e)}
    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}
