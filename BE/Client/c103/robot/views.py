# login/views.py
import json
import asyncio
import ssl
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from websockets import connect  # pip install websockets


@csrf_exempt
def robot_regist(request):
    """
    FE에서 JSON 형식으로 email, password를 POST로 전송하면,
    WebSocket 서버에 login 패킷을 전송한 후 그 결과를 반환합니다.
    """
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    email = data.get("email")
    access_token = data.get("access_token")
    robot_id = data.get("robot_id")

    if not access_token or not robot_id:
        return JsonResponse(
            {
                "status": "error",
                "message": f"Not enough user informations. {access_token} {robot_id}",
            },
            status=400,
        )

    # WebSocket 서버에 전송할 패킷 구성
    payload = {
        "action": "regist_robot",
        "email": email,
        "access_token": access_token,
        "robot_id": robot_id,
    }

    from c103.ws_manager import ws_manager  # 위에서 만든 매니저 임포트

    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    return JsonResponse(resp)


@csrf_exempt
def robot_location(request):
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    access_token = data.get("access_token")

    if not access_token:
        return JsonResponse(
            {
                "status": "error",
                "message": f"Not enough user informations. {access_token}",
            },
            status=400,
        )

    # WebSocket 서버에 전송할 패킷 구성
    payload = {
        "action": "request_location",
        "access_token": access_token,
    }

    from c103.ws_manager import ws_manager  # 위에서 만든 매니저 임포트

    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    return JsonResponse(resp)


@csrf_exempt
def robot_mode_change(request):
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    access_token = data.get("access_token")
    mode = data.get("mode")

    if not access_token:
        return JsonResponse(
            {
                "status": "error",
                "message": f"Not enough user informations. {access_token} {mode}",
            },
            status=400,
        )

    # WebSocket 서버에 전송할 패킷 구성
    payload = {
        "action": "mode_change",
        "access_token": access_token,
        "mode": mode,
    }

    from c103.ws_manager import ws_manager  # 위에서 만든 매니저 임포트

    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    return JsonResponse(resp)
