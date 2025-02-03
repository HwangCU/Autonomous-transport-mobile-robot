import serial
import time

# UART3 (ttyAMA3) 설정
ser = serial.Serial('/dev/ttyAMA3', baudrate=9600, timeout=1)

# distances = set()
distances = dict()

# 데이터 수신 및 출력
while True:
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        # print("Received:", data.decode('utf-8'))  # 받은 데이터 출력
        list_data = data.strip().split()
        print(list_data)
        # distances.update(list_data)
        # print(distances)
        for data in list_data:
            if data in distances:
                distances[data] += 1
            else:
                distances[data] = 1
        print(distances)
        time.sleep(0.1)
