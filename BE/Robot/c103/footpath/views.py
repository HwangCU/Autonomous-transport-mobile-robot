# footpath/views.py
import json
import asyncio
import ssl
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from websockets import connect  # pip install websockets


@csrf_exempt
def footpath_view(request):
    print("lon lat: ", request)
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
        print(data)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    y = data.get("y")
    x = data.get("x")
    access_token = data.get("access_token")

    if not x or not y:
        return JsonResponse(
            {"status": "error", "message": "longitude and latitude are required."},
            status=400,
        )

    # WebSocket 서버에 전송할 패킷 구성
    payload = {
        "action": "footpath",
        "x": x,
        "y": y,
        "access_token": access_token,
    }

    from c103.ws_manager import ws_manager  # 위에서 만든 매니저 임포트

    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    return JsonResponse(resp)
