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

    async def send_get_profile():
        # 개발 중 SSL 검증 비활성화 (운영 환경에서는 올바른 인증서 설정 필요)
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE

        # WebSocket 서버 URL (실제 환경에 맞게 수정)
        ws_url = "wss://c103.duckdns.org:8501"
        async with connect(ws_url, ssl=ssl_context) as websocket:
            await websocket.send(json.dumps(payload))
            response = await websocket.recv()
            return json.loads(response)

    # 동기 코드에서 asyncio 이벤트 루프 생성 후 실행
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    response_data = loop.run_until_complete(send_get_profile())
    loop.close()

    return JsonResponse(response_data)


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

    async def send_edit_profile():
        # 개발 중 SSL 검증 비활성화 (운영 환경에서는 올바른 인증서 설정 필요)
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE

        # WebSocket 서버 URL (실제 환경에 맞게 수정)
        ws_url = "wss://c103.duckdns.org:8501"
        async with connect(ws_url, ssl=ssl_context) as websocket:
            await websocket.send(json.dumps(payload))
            response = await websocket.recv()
            return json.loads(response)

    # 동기 코드에서 asyncio 이벤트 루프 생성 후 실행
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    response_data = loop.run_until_complete(send_edit_profile())
    loop.close()

    return JsonResponse(response_data)
