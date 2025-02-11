# footpath/views.py
import json
import asyncio
import ssl
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from websockets import connect  # pip install websockets


# Create your views here.


# TODO: 로봇 정보 전송
def response_gps_view():
    gps = {"longitude": 1, "latitude": 2}
    return gps


@csrf_exempt
def get_current_location_view(request):
    """
    로봇의 현재 위치를 응답하는 뷰.
    실제 구현에서는 로봇의 GPS 센서로부터 데이터를 읽어와야 합니다.
    여기서는 예시로 고정된 좌표를 반환합니다.
    """
    if request.method == "POST":
        # 여기서는 예시로 고정된 좌표를 사용
        current_latitude = 37.123456  # 예시 위도
        current_longitude = 127.654321  # 예시 경도

        return JsonResponse(
            {
                "status": "success",
                "latitude": current_latitude,
                "longitude": current_longitude,
            }
        )
    else:
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed"}, status=405
        )
