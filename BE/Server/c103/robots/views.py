# footpath/views.py
import json
import os
import json
import base64
import asyncio
from PIL import Image
from io import BytesIO

# 모델과 WebSocket 연결 정보 임포트
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from websockets import connect  # pip install websockets
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
        "status": "sucess",
        "action": "get_gps",
        "latitude": latitude,
        "longitude": longitude,
        "robot_id": robot_id,
    }

    try:
        # ORM 호출을 비동기 컨텍스트에서 실행xXa
        robot = await sync_to_async(Robot.objects.get)(robot_id=robot_id)
    except Robot.DoesNotExist:
        return JsonResponse(
            {"status": "error", "message": "Robot not found"}, status=404
        )
        # 로봇에게 전송
    user_id = robot.user_id

    try:
        ws, _ = client_connections[user_id]
        print(
            "connection info is ",
            ws,
            ws.remote_address,
        )
        resp = await ws.send(json.dumps(payload))
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
async def camera_view(request):
    if request.method == "POST":
        print("camera request: ", request)
        # 파일 받기
        file = request.FILES.get("frame")
        if not file:
            return JsonResponse(
                {"status": "error", "message": "No frame file provided."}, status=400
            )

        # POST 데이터에 포함된 JSON 문자열 읽기
        json_data = request.POST.get("json")
        if json_data:
            try:
                json_data = json.loads(json_data)
            except json.JSONDecodeError:
                return JsonResponse({"error": "Invalid JSON format"}, status=400)
        else:
            return JsonResponse({"error": "No JSON data provided"}, status=400)

        robot_id = json_data.get("robot_id")
        if not robot_id:
            return JsonResponse(
                {"status": "error", "message": "robot_id is required in JSON"},
                status=400,
            )

        try:
            robot = await sync_to_async(Robot.objects.get)(robot_id=robot_id)
        except Robot.DoesNotExist:
            return JsonResponse(
                {"status": "error", "message": "Robot not found."}, status=404
            )
        user_id = robot.user_id

        if user_id not in client_connections:
            return JsonResponse(
                {"status": "error", "message": "Client not connected."}, status=400
            )
        ws, ws_loop = client_connections[user_id]

        # 파일의 바이너리 데이터를 읽어서 Base64 인코딩
        file_data = file.read()  # InMemoryUploadedFile의 내용을 읽음
        camera_image = base64.b64encode(file_data).decode(
            "utf-8"
        )  # Base64로 인코딩하여 문자열로 변환

        # Base64로 인코딩된 이미지를 디코딩하여 이미지 객체로 변환
        image_data = base64.b64decode(camera_image)  # 다시 디코딩
        image = Image.open(BytesIO(image_data))
        image.show()  # 이미지를 화면에 출력

        # 전송할 패킷 구성
        payload = json.dumps(
            {
                "action": "camera_image",
                "image": camera_image,  # Base64 인코딩된 이미지
            }
        )

        print("WebSocket object:", ws)

        try:
            future = asyncio.run_coroutine_threadsafe(ws.send(payload), ws_loop)
            await asyncio.wrap_future(future)
            return JsonResponse(
                {"status": "success", "message": f"Successfully send image"}, status=200
            )

        except Exception as e:
            print("Error sending via WebSocket:", e)
            return JsonResponse(
                {"status": "error", "message": f"Failed to send image: {e}"}, status=500
            )
    return JsonResponse({"error": "Invalid request"}, status=405)
