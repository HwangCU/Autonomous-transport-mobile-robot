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
    # (2) WebSocket 서버에 "login" 패킷 전송
    resp = ws_manager.send_login(payload)

    return get_path(resp, des_x, des_y)


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
