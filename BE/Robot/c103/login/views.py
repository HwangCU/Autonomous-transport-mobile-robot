# login/views.py
import json
import asyncio
import ssl
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from websockets import connect  # pip install websockets


@csrf_exempt
def login_view(request):
    # print("login debug: ", request)
    """
    FE에서 JSON 형식으로 email, password를 POST로 전송하면,
    WebSocket 서버에 login 패킷을 전송한 후 그 결과를 반환합니다.
    """
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        print("1")
        data = json.loads(request.body)
        print("2")
        # print(data)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    robot_id = data.get("robot_id")
    password = data.get("password")
    if not robot_id or not password:
        return JsonResponse(
            {"status": "error", "message": "robot_id and password are required."},
            status=400,
        )

    # WebSocket 서버에 전송할 패킷 구성
    payload = {
        "action": "robot_login",
        "robot_id": robot_id,
        "password": password,
    }

    from c103.ws_manager import ws_manager  # 위에서 만든 매니저 임포트

    print("3")
    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)
    print("4")
    return JsonResponse(resp)
