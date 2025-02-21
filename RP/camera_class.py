import cv2
import requests
import time
from picamera2 import Picamera2
import json
from requests_toolbelt.multipart.encoder import MultipartEncoder



class CameraClass():
    def __init__(self):
        
        # Django 서버 주소
        self.SERVER_URL = "http://c103.duckdns.org:8502/api/camera/"

        # Picamera2 설정
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"size": (640, 480)}))
        self.picam2.start()

        time.sleep(2)  # 카메라 워밍업

        self.sending = False



    def send_image(self):
        """
        you must set 'self.sending = True' before you start this function
        """
        while self.sending:
            frame = self.picam2.capture_array()
            
            _, img_encoded = cv2.imencode('.jpg', frame)


            # 파일 및 JSON 데이터 준비
            files = {"frame": ("frame.jpg", img_encoded.tobytes(), "image/jpeg")}
            data = {"json": json.dumps({"robot_id": "user"})}  # JSON 데이터를 문자열로 변환하여 form-data에 포함

            try:
                # 파일과 JSON을 함께 전송 (data는 form-data로 전송됨)
                response = requests.post(self.SERVER_URL, files=files, data=data)

                print("Response Status Code:", response.status_code)
                print("Response Text:", response.text)
                
                response_json = response.json()
                print(response_json)
            except Exception as e:
                print(f"Error: {e}")

            time.sleep(0.1)  # 10FPS로 전송
    


if __name__ == '__name__':
    camera = CameraClass()
    camera.send_image()