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
from asgiref.sync import sync_to_async

from c103.ws_manager import send_to_client
from robots.models import Robot

from c103.Websocket.client_info import client_connections, client_locks, client_ip_list


@csrf_exempt
async def weight_view(request):
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
        weight = data.get("weight")
        robot_id = data.get("robot_id")
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    payload = {
        "action": "robot_weight",
        "weight": weight,
    }
    try:
        # ORM 호출을 비동기 컨텍스트에서 실행
        robot = await sync_to_async(Robot.objects.get)(robot_id=robot_id)
    except Robot.DoesNotExist:
        return JsonResponse(
            {"status": "error", "message": "Robot not found"}, status=404
        )

    user_id = robot.user_id

    # send_to_client는 비동기 함수이므로 await 사용
    await send_to_client(user_id, payload)

    return JsonResponse(payload)


@csrf_exempt
async def get_gps_view(request):
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
        "status": "fucking get_gps",
        "action": "get_gps",
        "latitude": latitude,
        "longitude": longitude,
        "robot_id": robot_id,
    }

    try:
        # ORM 호출을 비동기 컨텍스트에서 실행
        robot = await sync_to_async(Robot.objects.get)(robot_id=robot_id)
    except Robot.DoesNotExist:
        return JsonResponse(
            {"status": "error", "message": "Robot not found"}, status=404
        )
        # 로봇에게 전송
    user_id = robot.user_id

    try:
        print(
            "connection info is ",
            client_connections[user_id],
            client_connections[user_id].remote_address,
        )
        resp = await client_connections[user_id].send(json.dumps(payload))
        print(resp)
        return JsonResponse(
            {"status": "success", "message": f"Send GPS {latitude} {longitude}"}
        )
        # print("success")
    except Exception as e:
        return JsonResponse({"status": "error", "message": f"Send GPS Faile: {e}"})
        # print(f"error: {e}")


from datetime import datetime
import os

IMAGES_DIR = "~/media/camera_images"
os.makedirs(IMAGES_DIR, exist_ok=True)


@csrf_exempt
def camera_view(request):
    if request.method == "POST" and "frame" in request.FILES:
        # 현재 시간을 기준으로 파일명 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = f"frame_{timestamp}.jpg"
        filepath = os.path.join(IMAGES_DIR, filename)

        # 파일 저장
        with open(filepath, "wb+") as destination:
            for chunk in request.FILES["frame"].chunks():
                destination.write(chunk)

        # 최신 이미지 경로 업데이트
        latest_image_path = f"/media/camera_images/{filename}"

        """
        이미지 실험 예정
        robot = await sync_to_async(Robot.objects.get)(robot_id=robot_id)
        user_id = robot.user_id
        ws = client_connections[user_id]
        ws.send(json.dump(request))
        """

        return JsonResponse(
            {
                "status": "success",
                "message": "Image received and saved",
                "filename": filename,
            }
        )

    return JsonResponse({"status": "error", "message": "Invalid request"}, status=400)

    # TODO: footpath 가공해서 로봇에서 받을 수 있게 수정하

    # payload = {
    #     "action": "get_gps",
    #     "latitude": latitude,
    #     "longitude": longitude,
    #     "robot_id": robot_id,
    # }

    print("camera_frame : ", frame)

    # (2) WebSocket 서버에 "login" 패킷 전송

    return JsonResponse({"status": "success"})
