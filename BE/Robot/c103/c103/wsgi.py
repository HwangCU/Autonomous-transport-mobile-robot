# c103/c103/wsgi.py
"""
WSGI config for c103 project.
"""
import os
from django.core.wsgi import get_wsgi_application

# 여기서 ws_manager 임포트
from .ws_manager import ws_manager

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "c103.settings")

# 1) WebSocket 연결 시도
ws_manager.connect()

# 2) WSGI 애플리케이션
application = get_wsgi_application()
