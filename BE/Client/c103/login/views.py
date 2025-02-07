# login/views.py
import json
import asyncio
import ssl
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from websockets import connect  # pip install websockets


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

    async def send_login():
        # 개발 중 SSL 검증 비활성화 (운영 환경에서는 올바른 인증서 설정 필요)
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE

        # WebSocket 서버 URL (실제 환경에 맞게 수정)
        ws_url = "wss://192.168.100.40:8001"
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

    async def send_login():
        # 개발 중 SSL 검증 비활성화 (운영 환경에서는 올바른 인증서 설정 필요)
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE

        # WebSocket 서버 URL (실제 환경에 맞게 수정)
        ws_url = "wss://192.168.100.40:8001"
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
