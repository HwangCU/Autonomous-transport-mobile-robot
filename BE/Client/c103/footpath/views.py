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
from c103.ws_manager import ws_manager  # 위에서 만든 매니저 임포트
import math
import json

from vars import LAST_GPS


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

    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    return JsonResponse(resp)


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
    # lg = json.loads(LAST_GPS)
    start_x = LAST_GPS.get("longitude")
    start_y = LAST_GPS.get("latitude")

    print("\n 데이터 검증 : ", start_x, start_y)

    return get_path(start_x, start_y, des_x, des_y)


@csrf_exempt
def home_sweet_home_view(request):
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

    access_token = data.get("access_token")

    if not access_token:
        return JsonResponse(
            {"status": "error", "message": "dest and access_token are required."},
            status=400,
        )

    # WebSocket 서버에 전송할 패킷 구성
    payload = {
        "action": "get_profile",
        "access_token": access_token,
    }

    from c103.ws_manager import ws_manager  # 위에서 만든 매니저 임포트

    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    print(resp)
    dest = resp.get("profile").get("address")

    des_x, des_y = get_cord_by_road(dest)
    # lg = json.loads(LAST_GPS)
    start_x = LAST_GPS.get("longitude")
    start_y = LAST_GPS.get("latitude")

    print("\n 데이터 검증 : ", start_x, start_y)

    return get_path(start_x, start_y, des_x, des_y)


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
    print("\ncoordi data is ", coordinate_data)
    lat = coordinate_data["newLat"]
    lon = coordinate_data["newLon"]
    return (
        lat,
        lon,
    )


def get_path(start_x, start_y, des_x, des_y):
    origin = url_encode("출발지")
    destination = url_encode("도착지")

    url = (
        "https://apis.openapi.sk.com/tmap/routes/pedestrian?version=1&callback=function"
    )

    print(
        " 좌표 검증 ",
        start_x,
        start_y,
        des_y,
        des_x,
    )

    payload = {
        "startX": start_x,
        "startY": start_y,
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
    waypoints = []
    route_coordinates = []
    total_time = 0

    for feature in response["features"]:
        if feature["geometry"]["type"] == "LineString":
            route_coordinates.extend(feature["geometry"]["coordinates"])
            for coord in feature["geometry"]["coordinates"]:
                waypoints.append(
                    {"latitude": coord[1], "longitude": coord[0]}  # 위도  # 경도
                )
            if "time" in feature["properties"]:
                total_time += feature["properties"]["time"]

    return JsonResponse(
        {
            "status": "success",
            "message": "path success",
            "path_list": route_coordinates,
            "waypoints": waypoints,
            "time": total_time,
        }
    )


@csrf_exempt
def send_footpath_view(request):
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
    path_list = data.get("path_list")

    # TODO: footpath 가공해서 로봇에서 받을 수 있게 수정하
    def convert_path_list_to_waypoints(path_list):
        """
        path_list: [[경도, 위도], [경도, 위도], ...]
        출력: [{"latitude": 위도, "longitude": 경도, "yaw": yaw}, ...]
        """
        waypoints = []
        for i, point in enumerate(path_list):
            # point[0]: 경도, point[1]: 위도
            if i < len(path_list) - 1:
                # 다음 점과의 차이로 yaw(라디안)를 계산
                dx = path_list[i + 1][0] - point[0]
                dy = path_list[i + 1][1] - point[1]
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0  # 마지막 점은 0.0으로 설정 (원하는 값으로 조정 가능)

            waypoint = {"latitude": point[1], "longitude": point[0], "yaw": yaw}
            waypoints.append(waypoint)
        return waypoints

    # 변환 실행
    waypoints = convert_path_list_to_waypoints(path_list)

    # YAML 형식과 유사하게 출력 (여기서는 JSON으로 출력)
    output_data = {"waypoints": waypoints}
    print(json.dumps(output_data, indent=2))
    #

    payload = {
        "action": "send_footpath",
        "access_token": access_token,
        "footpath": waypoints,
    }

    print("send footpath : ", payload)
    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    return JsonResponse(resp)
