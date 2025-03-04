import os
from dotenv import load_dotenv
import requests

# .env 파일 로드
load_dotenv()

# 환경 변수 확인
api_key = os.getenv('TMAP_API_KEY')

url = "https://apis.openapi.sk.com/tmap/routes/pedestrian?version=1&callback=function"

payload = {
    "startX": 126.92365493654832,
    "startY": 37.556770374096615,
    "angle": 20,
    "speed": 30,
    "endPoiId": "10001",
    "endX": 126.92432158129688,
    "endY": 37.55279861528311,
    "passList": "126.92774822,37.55395475_126.92577620,37.55337145",
    "reqCoordType": "WGS84GEO",
    "startName": "%EC%B6%9C%EB%B0%9C",
    "endName": "%EB%8F%84%EC%B0%A9",
    "searchOption": "0",
    "resCoordType": "WGS84GEO",
    "sort": "index"
}

headers = {
    "accept": "application/json",
    "content-type": "application/json",
    "appKey": api_key
}

response = requests.post(url, json=payload, headers=headers)

print(f"Response status code: {response.status_code}")  # HTTP 상태 코드 확인
print(response.text)