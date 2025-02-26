import torch
import cv2
import numpy as np
# 통신 & cam lib
import time
import requests
from picamera2 import Picamera2
import json
from requests_toolbelt.multipart.encoder import MultipartEncoder

# Django 서버 주소
SERVER_URL = "http://c103.duckdns.org:8502/api/camera/"

# Picamera2 설정
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2)  # 카메라 워밍업

# YOLOv5 모델 불러오기
model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5s.pt', force_reload=True)

# 화면 출력 설정
cv2.namedWindow("YOLOv5 Object Detection", cv2.WINDOW_NORMAL)

while True:
    # 프레임 읽기
    # ret, frame = cap.read()
    frame = picam2.capture_array()
    if not ret:
        print("Failed to grab frame")
        break

    # YOLOv5 모델에 입력
    results = model(frame)

    # 탐지 결과에서 바운딩 박스와 라벨 추출
    for result in results.xyxy[0]:
        x1, y1, x2, y2, conf, cls = result

        # 바운딩 박스 좌표 및 클래스 정보
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        label = f"{model.names[int(cls)]} {conf:.2f}"

        # 바운딩 박스 그리기
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 결과 화면 출력
    cv2.imshow("YOLOv5 Object Detection", frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cv2.destroyAllWindows()
