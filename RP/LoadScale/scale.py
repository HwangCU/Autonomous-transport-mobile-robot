import sys
import RPi.GPIO as GPIO
from .hx711 import HX711

class LoadScale:
    def __init__(self, dt_pin, sck_pin, reference_unit=8600):
        self.dt_pin = dt_pin
        self.sck_pin = sck_pin
        self.reference_unit = reference_unit
        
        # HX711 초기화
        self.hx = HX711(dout=self.dt_pin, pd_sck=self.sck_pin)
        self.hx.set_reading_format("MSB", "MSB")
        self.hx.set_reference_unit(self.reference_unit)
        
        # 초기화 및 영점 조정
        self.hx.reset()
        self.hx.tare(30)
        
        # 드리프트 보정 변수
        self.drift_offset = 0
        self.drift_learning_rate = 0.001
        
        # 저역통과 필터 설정
        self.alpha = 0.8
        self.filtered_value = self.hx.get_weight(5)
        
        print("Tare done! Add weight now...")

    def update_drift(self, measured, filtered):
        # 물체가 올려진 상태에서 무게가 거의 일정하다고 가정할 수 있다면,
        # 측정된 값과 필터 결과의 차이를 천천히 추정 오프셋으로 반영
        error = filtered - measured
        self.drift_offset += self.drift_learning_rate * error

    def scale_read(self) -> int:
        val = self.hx.get_weight(5)

        # 드리프트 오차 보정
        val -= self.drift_offset

        # 저역 통과 필터
        self.filtered_value = self.alpha * val + (1 - self.alpha) * self.filtered_value

        self.update_drift(val, self.filtered_value)

        # 값 변환 및 음수 예외처리
        kilogram = round(self.filtered_value, 1)
        
        if kilogram <= 0.0:
            if kilogram > -0.5:
                kilogram = 0.0
            else:
                print("Waitting for Retare...")
                self.hx.tare(30)
                print("Tare done!")

        self.hx.power_down()
        self.hx.power_up()

        return int(kilogram)

    def cleanAndExit(self):
        print("Cleaning...")
        print("Scaler turned off!")
        sys.exit()
