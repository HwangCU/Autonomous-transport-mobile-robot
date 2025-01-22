import asyncio
import json
import ssl
import websockets


async def register_user():
    uri = "wss://localhost:8001"

    # TLS 설정
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ssl_context.check_hostname = False
    ssl_context.verify_mode = ssl.CERT_NONE

    async with websockets.connect(uri, ssl=ssl_context) as websocket:
        # 회원가입 데이터 전송
        data = {
            "action": "register",
            "username": "testuser",
            "password": "securepassword",
            "name": "Test User",
            "email": "testuser@example.com",
        }
        await websocket.send(json.dumps(data))
        print("Sent:", data)

        # 서버 응답 수신
        response = await websocket.recv()
        print("Received:", response)


if __name__ == "__main__":
    asyncio.run(register_user())
