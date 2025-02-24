import Jetson.GPIO as GPIO
import time

# 테스트할 GPIO 핀 번호
test_pin = 29  # 사용하려는 실제 핀 번호로 변경

def main():
    # GPIO 모드 설정
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(test_pin, GPIO.OUT)
    
    try:
        print("GPIO 테스트 시작...")
        for _ in range(5):  # 5번 반복
            print("HIGH")
            GPIO.output(test_pin, GPIO.HIGH)
            time.sleep(1)
            print("LOW")
            GPIO.output(test_pin, GPIO.LOW)
            time.sleep(1)
            
    except Exception as e:
        print(f"에러 발생: {e}")
    
    finally:
        GPIO.cleanup()
        print("GPIO cleanup completed")

if __name__ == '__main__':
    main()