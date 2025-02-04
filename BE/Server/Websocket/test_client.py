import asyncio
import json
import ssl
import websockets  # pip install websockets


async def test_client():
    # 개발 단계에서는 SSL 인증서를 검증하지 않도록 설정 (운영 시에는 적절히 설정)
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ssl_context.check_hostname = False
    ssl_context.verify_mode = ssl.CERT_NONE

    ws_url = "wss://localhost:8001"  # WebSocket 서버 URL (환경에 맞게 수정)

    async with websockets.connect(ws_url, ssl=ssl_context) as websocket:
        # 1. 로그인 요청 보내기

        register_payload = {
            "action": "register",
            "username": "test2user",
            "password": "test2pass",
            "email": "email2@email.com",
            "address": "address",
        }
        print("Sending register payload:", register_payload)
        await websocket.send(json.dumps(register_payload))

        response = await websocket.recv()
        register_response = json.loads(response)
        print("Register response:", register_response)

        login_payload = {
            "action": "login",
            "username": "test2user",
            "password": "test2pass",
        }
        print("Sending login payload:", login_payload)
        await websocket.send(json.dumps(login_payload))

        response = await websocket.recv()
        login_response = json.loads(response)
        print("Login response:", login_response)
        access_token = login_response.get("access_token")

        # 2. 만약 로봇 등록이 필요하면, 로봇 등록 요청 보내기
        if login_response.get("require_robot"):
            robot_payload = {
                "action": "register_robot",
                "robot_id": "test_robot_002",
                "access_token": access_token,
            }
            print("Sending robot registration payload:", robot_payload)
            await websocket.send(json.dumps(robot_payload))

            robot_response = await websocket.recv()
            print("Robot registration response:", robot_response)
        else:
            print("Robot already registered.")


if __name__ == "__main__":
    asyncio.run(test_client())
