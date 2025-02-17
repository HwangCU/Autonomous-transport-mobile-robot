import json
import asyncio
from robot_info import robot_connections, robot_locks


async def handle_request_robot(payload, user):
    """WebSocket을 통해 로봇에게 요청을 보내고 응답을 받음."""
    if user.id not in robot_connections:
        return {"status": "error", "message": f"Robot {user.id} not connected"}

    if user.id not in robot_locks:
        robot_locks[user.id] = asyncio.Lock()  # Lock이 없으면 생성

    async with robot_locks[user.id]:  # 🔒 Lock을 사용하여 동시 recv() 방지
        try:
            await robot_connections[user.id].send(json.dumps(payload))
            try:
                robot_response = await asyncio.wait_for(
                    robot_connections[user.id].recv(), timeout=10
                )
                return {
                    "status": "success",
                    "robot_location": json.loads(robot_response),
                }
            except asyncio.TimeoutError:
                return {
                    "status": "error",
                    "message": "Robot did not respond in time",
                }
        except Exception as e:
            return {"status": "error", "message": f"Internal server error: {e}"}
