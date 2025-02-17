# login/views.py
import json
import asyncio
import ssl
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from websockets import connect  # pip install websockets


@csrf_exempt
def get_profile_view(request):
    print("login debug: ", request)

    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
        print(data)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    access_token = data.get("access_token")
    if not access_token:
        return JsonResponse(
            {"status": "error", "message": "email and password are required."},
            status=400,
        )

    # WebSocket 서버에 전송할 패킷 구성
    payload = {
        "action": "get_profile",
        "access_token": access_token,
    }

    from c103.ws_manager import ws_manager  # 위에서 만든 매니저 임포트

    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    return JsonResponse(resp)


@csrf_exempt
def edit_profile_view(request):
    print("login debug: ", request)

    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
        print(data)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    access_token = data.get("access_token")
    address = data.get("address")
    username = data.get("username")
    if not access_token:
        return JsonResponse(
            {"status": "error", "message": "email and password are required."},
            status=400,
        )

    # WebSocket 서버에 전송할 패킷 구성
    payload = {
        "action": "edit_profile",
        "access_token": access_token,
        "address": address,
        "username": username,
    }

    from c103.ws_manager import ws_manager  # 위에서 만든 매니저 임포트

    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    return JsonResponse(resp)
