## 사전 준비

1. **가상환경(venv) 권장**:  
   ```bash
   python -m venv venv
   source venv/bin/activate   # Windows는 venv\Scripts\activate
   ```
2. **라이브러리 설치**:  
   ```bash
   pip install -r requirements.txt
   ```

## 설정 파일 설명

- **`jwt_key.pem`**:  
  - Base64로 인코딩된 랜덤 바이트 시크릿 키.  
  - 생성 예시
  ```bash
  openssl rand -base64 32 > jwt_key.pem
  ```

- **`cert.pem` / `key.pem`**:  
  - TLS 통신(WSS)을 위한 인증서와 비밀키 파일.  
  - 생성 예시
  ```bash
  openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 365 -nodes
  # CN에는 도메인 또는 IP 주소 입력 필요
  ```


---

## 실행 방법
1. **서버 실행**:  
   - (프로젝트 루트에서) 아래 명령어 실행:
     ```bash
     python server_main.py
     ```
   - 서버가 정상적으로 실행되면 콘솔에  
     ```
     Secure WebSocket server running on wss://0.0.0.0:8001
     ```
     메시지가 표시됩니다.
---

## WebSocket 통신 형식

### 1) 회원가입

- **Action**: `"register"`
- **Request(JSON)**:
  ```json
  {
    "action": "register",
    "username": "example_user",
    "password": "secure_password",
    "name": "홍길동",
    "email": "hong@example.com"
  }
  ```
- **Response(JSON)** 성공 예시:
  ```json
  {
    "status": "success",
    "message": "User registered successfully"
  }
  ```
- **에러**(예: 중복, 필드 누락 등):
  ```json
  {
    "status": "error",
    "message": "Username already exists"
  }
  ```

### 2) 로그인

- **Action**: `"login"`
- **Request(JSON)**:
  ```json
  {
    "action": "login",
    "username": "example_user",
    "password": "secure_password"
  }
  ```
- **Response(JSON)** 성공 예시:
  ```json
  {
    "status": "success",
    "message": "Login successful",
    "access_token": "<JWT_ACCESS_TOKEN>",
    "refresh_token": "<JWT_REFRESH_TOKEN>"
  }
  ```
- **에러**:
  ```json
  {
    "status": "error",
    "message": "Invalid username or password"
  }
  ```

---

## 테스트 (예시 코드)

아래는 간단한 **클라이언트** 예시(`client_test.py` 등)로, **WebSocket**에 연결해 테스트합니다.

```python
import asyncio
import json
import ssl
import websockets

async def test_register():
    uri = "wss://{ip_addr}:8001"
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ssl_context.check_hostname = False
    ssl_context.verify_mode = ssl.CERT_NONE

    async with websockets.connect(uri, ssl=ssl_context) as websocket:
        register_data = {
            "action": "register",
            "username": "testuser",
            "password": "testpass",
            "name": "Tester",
            "email": "test@example.com"
        }
        await websocket.send(json.dumps(register_data))
        response = await websocket.recv()
        print("Register Response:", response)

async def test_login():
    uri = "wss://{ip_addr}:8001"
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ssl_context.check_hostname = False
    ssl_context.verify_mode = ssl.CERT_NONE

    async with websockets.connect(uri, ssl=ssl_context) as websocket:
        login_data = {
            "action": "login",
            "username": "testuser",
            "password": "testpass"
        }
        await websocket.send(json.dumps(login_data))
        response = await websocket.recv()
        print("Login Response:", response)

async def main():
    print("Testing register...")
    await test_register()
    print("Testing login...")
    await test_login()

if __name__ == "__main__":
    asyncio.run(main())
```