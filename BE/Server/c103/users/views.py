from django.http import JsonResponse
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
        return {"status": "success", "message": "Email verified successfully"}
    except User.DoesNotExist:
        return {"status": "error", "message": "User not found"}


@csrf_exempt
async def verify_email(request, token):
    """이메일 인증 처리"""
    print("\n\n이메일 인증 처리 수행\n\n")
    email = verify_email_token(token)
    if email:
        response = await activate_user(email)
        return JsonResponse(response)
    else:
        return JsonResponse(
            {"status": "error", "message": "Invalid or expired token"}, status=400
        )
