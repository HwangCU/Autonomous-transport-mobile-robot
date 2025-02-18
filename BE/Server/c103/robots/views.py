# footpath/views.py
import json
import asyncio
import ssl
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from websockets import connect  # pip install websockets
import urllib.parse
import requests
from rest_framework.response import Response


@csrf_exempt
def weight_view(request):
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
        print(data)
        weight = data.get("weight")
        robot_id = data.get("robot_id")
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    access_token = data.get("access_token")

    payload = {
        "action": "request_location",
        "access_token": access_token,
    }
    print(payload)

    return JsonResponse(payload)


@csrf_exempt
def get_gps_view(request):
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
        print(data)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    latitude = data.get("latitude")
    longitude = data.get("longitude")
    robot_id = data.get("robot_id")

    # TODO: footpath 가공해서 로봇에서 받을 수 있게 수정하

    payload = {
        "action": "get_gps",
        "latitude": latitude,
        "longitude": longitude,
        "robot_id": robot_id,
    }

    print("gps payload : ", payload)
    print("latitude: ", latitude, "  longitude : ", longitude)

    # (2) WebSocket 서버에 "login" 패킷 전송

    return JsonResponse(payload)
