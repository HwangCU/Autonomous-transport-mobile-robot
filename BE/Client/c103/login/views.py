# login/views.py
import json
import asyncio
import ssl
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from websockets import connect  # pip install websockets
from c103.ws_manager import ws_manager  # 위에서 만든 매니저 임포트


@csrf_exempt
def login_view(request):
    print("login debug: ", request)
    """
    FE에서 JSON 형식으로 email, password를 POST로 전송하면,
    WebSocket 서버에 login 패킷을 전송한 후 그 결과를 반환합니다.
    """
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    # (1) 만약 아직 연결 안 되어 있으면 connect()
    if not ws_manager.is_connected():
        ws_manager.connect()

    try:
        data = json.loads(request.body)
        print(data)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    email = data.get("email")
    password = data.get("password")

    if not email or not password:
        return JsonResponse(
            {"status": "error", "message": "email and password are required."},
            status=400,
        )

    # WebSocket 서버에 전송할 패킷 구성
    payload = {
        "action": "login",
        "email": email,
        "password": password,
    }

    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    return JsonResponse(resp)


@csrf_exempt
def sns_login_view(request):
    """
    FE에서 JSON 형식으로 email, password를 POST로 전송하면,
    WebSocket 서버에 login 패킷을 전송한 후 그 결과를 반환합니다.
    """
    print("\n", request, "\n")
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    email = data.get("email")
    usernum = data.get("usernum")
    userloginresource = data.get("userloginresource")
    token = data.get("token")

    print(email, usernum, userloginresource, token)
    if not email or not usernum or not userloginresource or not token:
        return JsonResponse(
            {
                "status": "error",
                "message": f"not enough information. {email} {usernum} {userloginresource} {token}.",
            },
            status=400,
        )

    # WebSocket 서버에 전송할 패킷 구성
    payload = {
        "action": "sns_login",
        "email": email,
        "usernum": usernum,
        "userloginresource": userloginresource,
        "token": token,
    }

    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    return JsonResponse(resp)
