import time
import sys
import RPi.GPIO as GPIO
from hx711 import HX711


def cleanAndExit():
    print("Cleaning...")
        
    print("Scaler turned off!")
    sys.exit()

DT = 5
SCK = 6

hx = HX711(dout=DT, pd_sck=SCK)


hx.set_reading_format("MSB", "MSB")


referenceUnit = 8600
hx.set_reference_unit(referenceUnit)

hx.reset()

hx.tare(30)

drift_offset = 0
drift_learning_rate = 0.001  # 드리프트 추정에 사용하는 느린 적응 속도

def update_drift(measured, filtered):
    global drift_offset
    # 물체가 올려진 상태에서 무게가 거의 일정하다고 가정할 수 있다면,
    # 측정된 값과 필터 결과의 차이를 천천히 추정 오프셋으로 반영
    error = filtered - measured
    drift_offset += drift_learning_rate * error

# 저역통과 필터 설정
# 작을수록 더 강한 필터링(느린 응답) 0~1 사이이
alpha = 0.8  # (alpha)% 반영
filtered_value = hx.get_weight(5)

print("Tare done! Add weight now...")

while True:
    try:
        val = hx.get_weight(5)

        # 드리프트 오차 보정
        val -= drift_offset

        # 저역 통과 필터
        filtered_value = alpha * val + (1 - alpha) * filtered_value

        update_drift(val, filtered_value)

        # 값 변환 및 음수 예외처리리
        kilogram = round(filtered_value, 1)
        
        if kilogram <= 0.0:
            if kilogram > -0.5:
                kilogram = 0.0
            else:
                print("Waitting for Retare...")
                hx.tare(30)
                print("Tare done!")

        print(kilogram)

        hx.power_down()
        hx.power_up()
        time.sleep(0.1)

    except (KeyboardInterrupt, SystemExit):
        cleanAndExit()
