# global_vars.py
client_connections = {}  # Key: robot_id, Value: WebSocket 연결 객체
client_locks = {}  # Key: robot_id, Value: asyncio.Lock 객체
client_ip_list = {}
