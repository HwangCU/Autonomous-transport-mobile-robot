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
        data = json.loads(request.body)
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

    async def send_login():
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
    response_data = loop.run_until_complete(send_login())
    loop.close()

    return JsonResponse(response_data)
