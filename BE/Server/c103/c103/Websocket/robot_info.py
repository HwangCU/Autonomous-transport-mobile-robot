# global_vars.py
robot_connections = {}  # Key: robot_id, Value: WebSocket 연결 객체
robot_locks = {}  # Key: robot_id, Value: asyncio.Lock 객체
robot_ip_list = {}
