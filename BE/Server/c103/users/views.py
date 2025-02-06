from django.shortcuts import render
from django.views.decorators.csrf import csrf_exempt
from users.models import User
from c103.utils import verify_email_token
from asgiref.sync import sync_to_async


@sync_to_async
def activate_user(email):
    """사용자 계정 활성화"""
    try:
        user = User.objects.get(email=email)
        user.is_active = True
        user.save()
        return {"status": "success"}
    except User.DoesNotExist:
        return {"status": "error"}


@csrf_exempt
async def verify_email(request, token):
    """이메일 인증 처리 및 HTML 반환"""
    print("이메일 인증 처리 수행\n\n")
    email = verify_email_token(token)

    if email:
        response = await activate_user(email)
        if response["status"] == "success":
            return render(
                request, "verify_email/success.html"
            )  # ✅ 성공 시 success.html 반환
        else:
            return {"status": "error"}  # ✅ 실패 시 error.html 반환

    return {"status": "error"}  # ✅ 인증 실패 시 error.html 반환
