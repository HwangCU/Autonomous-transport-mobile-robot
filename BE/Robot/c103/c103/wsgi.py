import os
from django.core.wsgi import get_wsgi_application

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "c103.settings")

application = get_wsgi_application()

# Autoreload 등 중복 실행 방지
if os.environ.get("RUN_MAIN") == "true":
    from c103 import ws_manager

    ws_manager.WSManager.connect()
