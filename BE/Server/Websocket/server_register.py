from django.core.mail import send_mail
from django.conf import settings
from asgiref.sync import sync_to_async
from django.core.exceptions import ValidationError
from django.contrib.auth.hashers import make_password
from users.models import User
from c103.utils import generate_email_token


@sync_to_async
def create_user(data):
    """비동기로 사용자 생성 (is_active=False)"""
    user = User.objects.create(
        username=data["username"],
        password=make_password(data["password"]),
        email=data["email"],
        address=data["address"],
        is_active=False,  # 이메일 인증 전까지 비활성화
    )
    user.save()


@sync_to_async
def check_user_exists(field, value):
    """필드 중복 확인"""
    return User.objects.filter(**{field: value}).exists()


@sync_to_async
def send_verification_email(user):
    """이메일 인증 링크 전송"""
    token = generate_email_token(user.email)
    print("토큰은 토큰 : ", token)
    verification_url = f"http://192.168.100.40:8002/api/verify-email/{token}"  #

    subject = "이메일 인증을 완료해주세요"
    message = f"안녕하세요, {user.username}님!\n\n아래 링크를 클릭하여 이메일 인증을 완료해주세요:\n\n{verification_url}"
    send_mail(subject, message, settings.DEFAULT_FROM_EMAIL, [user.email])


async def handle_registration(data):
    """회원가입 처리 + 이메일 인증 추가"""
    try:
        required_fields = {"username", "password", "email", "address"}
        if not required_fields.issubset(data.keys()):
            return {"status": "error", "message": "Missing required fields"}

        if await check_user_exists("username", data["username"]):
            return {"status": "error", "message": "Username already exists"}
        if await check_user_exists("email", data["email"]):
            return {"status": "error", "message": "Email already exists"}

        await create_user(data)  # 사용자 생성 (비활성화 상태)
        user = await User.objects.aget(email=data["email"])  # 비동기로 사용자 가져오기
        await send_verification_email(user)  # 이메일 인증 링크 전송

        return {
            "status": "success",
            "message": "User registered. Check your email for verification",
        }

    except ValidationError as e:
        return {"status": "error", "message": str(e)}
    except Exception as e:
        return {"status": "error", "message": f"Internal server error: {e}"}
