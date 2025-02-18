import os
import threading
from django.core.wsgi import get_wsgi_application
from c103.ws_manager import ws_manager, run_tcp_server

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "c103.settings")


def start_threads():
    # 1) 웹소켓 매니저 연결 (스레드로 돌려도 되고, 직접 호출도 가능)
    threading.Thread(target=ws_manager.connect, daemon=True).start()
    # 2) TCP 서버 스레드
    threading.Thread(target=run_tcp_server, daemon=True).start()
    print("[WSGI] Started ws_manager.connect + run_tcp_server threads")


# 중복 실행 방지 (Django autoreload 시 2번 실행될 수 있음)
if os.environ.get("RUN_MAIN") == "true":
    start_threads()

application = get_wsgi_application()
