# footpath/views.py
import json
import asyncio
import ssl
from django.http import JsonResponse
from django.views.decorators.csrf import csrf_exempt
from websockets import connect  # pip install websockets
import urllib.parse
import requests
from rest_framework.response import Response


@csrf_exempt
def request_location_view(request):
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
        print(data)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    access_token = data.get("access_token")

    payload = {
        "action": "request_location",
        "access_token": access_token,
    }

    async def send_footpath():
        # 개발 중 SSL 검증 비활성화 (운영 환경에서는 올바른 인증서 설정 필요)
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE

        # WebSocket 서버 URL (실제 환경에 맞게 수정)
        ws_url = "wss://c103.duckdns.org:8501"
        async with connect(ws_url, ssl=ssl_context) as websocket:
            await websocket.send(json.dumps(payload))
            response = await websocket.recv()
            return json.loads(response)

    # 동기 코드에서 asyncio 이벤트 루프 생성 후 실행
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    response_data = loop.run_until_complete(send_footpath())
    loop.close()

    return JsonResponse(response_data)


# TODO: 앱으로부터 목표(x, y) 받아오기,
@csrf_exempt
def footpath_view(request):
    print("lon lat: ", request)
    if request.method != "POST":
        return JsonResponse(
            {"status": "error", "message": "Only POST method allowed."}, status=405
        )

    try:
        data = json.loads(request.body)
        print(data)
    except json.JSONDecodeError:
        return JsonResponse({"status": "error", "message": "Invalid JSON."}, status=400)

    dest = data.get("destination")
    access_token = data.get("access_token")

    if not dest or not access_token:
        return JsonResponse(
            {"status": "error", "message": "dest and access_token are required."},
            status=400,
        )

    des_x, des_y = get_cord_by_road(dest)

    payload = {
        "action": "request_location",
        "access_token": access_token,
    }

    async def send_footpath():
        # 개발 중 SSL 검증 비활성화 (운영 환경에서는 올바른 인증서 설정 필요)
        ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_context.check_hostname = False
        ssl_context.verify_mode = ssl.CERT_NONE

        # WebSocket 서버 URL (실제 환경에 맞게 수정)
        ws_url = "wss://c103.duckdns.org:8501"
        async with connect(ws_url, ssl=ssl_context) as websocket:
            await websocket.send(json.dumps(payload))
            response = await websocket.recv()
            return json.loads(response)

    # 동기 코드에서 asyncio 이벤트 루프 생성 후 실행
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    response_data = loop.run_until_complete(send_footpath())
    loop.close()

    return get_path(response_data, des_x, des_y)


def url_encode(text):
    """UTF-8 기반 URL 인코딩"""
    return urllib.parse.quote(text, encoding="utf-8")


def get_cord_by_road(dest):
    fulladdr = url_encode(dest)
    url = f"https://apis.openapi.sk.com/tmap/geo/fullAddrGeo?addressFlag=F02&version=1&fullAddr={fulladdr}"

    headers = {
        "accept": "application/json",
        "appKey": "5JFV40Cr66417WNN05G5K1bV6AL1BNVEaymIjAbF",
    }

    response = requests.get(url, headers=headers).json()

    print(response)

    coordinate_data = response["coordinateInfo"]["coordinate"][0]  # 첫 번째 요소
    lat = coordinate_data["newLat"]
    lon = coordinate_data["newLon"]
    return (
        lat,
        lon,
    )


def get_path(response_data, des_x, des_y):
    origin = url_encode("출발지")
    destination = url_encode("도착지")

    url = (
        "https://apis.openapi.sk.com/tmap/routes/pedestrian?version=1&callback=function"
    )

    print(
        " 좌표 검증 ",
        response_data["latitude"],
        response_data["longitude"],
        des_x,
        des_y,
    )

    payload = {
        "startX": response_data["longitude"],
        "startY": response_data["latitude"],
        "endX": des_y,
        "endY": des_x,
        "startName": origin,
        "endName": destination,
    }
    headers = {
        "accept": "application/json",
        "appKey": "5JFV40Cr66417WNN05G5K1bV6AL1BNVEaymIjAbF",
        "content-type": "application/json",
    }
    try:
        response = requests.post(url, json=payload, headers=headers).json()
    except Exception as e:
        return JsonResponse(
            {"status": "error", "message": f"SKT API Error: {e}"}, status=500
        )

    print("\n\n final response data \n\n", response)

    # 모든 이동 경로를 저장할 리스트
    route_coordinates = []

    # features 리스트에서 데이터 추출
    total_time = 0

    for feature in response["features"]:
        if feature["geometry"]["type"] == "LineString":
            route_coordinates.extend(feature["geometry"]["coordinates"])
            if "time" in feature["properties"]:
                total_time += feature["properties"]["time"]

        

    print(route_coordinates)

    return JsonResponse(
        {
            "status": "success",
            "message": "path success",
            "path_list": route_coordinates,
            "time":total_time,
        }
    )
