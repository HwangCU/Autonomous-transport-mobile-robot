import Jetson.GPIO as GPIO
import time

# 핀 설정
ENA_pin = 33  # PWM pin for Jetson Orin Nano
IN1_pin = 29  # Direction control 1
IN2_pin = 31  # Direction control 2

def setup():
    # GPIO 모드 설정
    GPIO.setmode(GPIO.BOARD)
    
    # 핀 설정
    GPIO.setup(ENA_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1_pin, GPIO.OUT)
    GPIO.setup(IN2_pin, GPIO.OUT)
    # PWM 객체 생성
    pwm = GPIO.PWM(ENA_pin, 1000)  # 1KHz frequency
    return pwm

def forward(pwm, speed):
    GPIO.output(IN1_pin, GPIO.HIGH)
    GPIO.output(IN2_pin, GPIO.LOW)
    print(f"IN1 status: {GPIO.input(IN1_pin)}")
    print(f"IN2 status: {GPIO.input(IN2_pin)}")
    pwm.ChangeDutyCycle(speed)

def reverse(pwm, speed):
    GPIO.output(IN1_pin, GPIO.LOW)
    GPIO.output(IN2_pin, GPIO.HIGH)
    print(f"IN1 status: {GPIO.input(IN1_pin)}")
    print(f"IN2 status: {GPIO.input(IN2_pin)}")
    pwm.ChangeDutyCycle(speed)

def stop(pwm):
    GPIO.output(IN1_pin, GPIO.LOW)
    GPIO.output(IN2_pin, GPIO.LOW)
    print(f"IN1 status: {GPIO.input(IN1_pin)}")
    print(f"IN2 status: {GPIO.input(IN2_pin)}")
    pwm.ChangeDutyCycle(0)

def main():
    pwm = setup()
    pwm.start(0)
    try:
        print("Starting motor test...")
        
        # 정방향 테스트
        print("Forward...")
        forward(pwm, 50)  # 50% 속도
        time.sleep(2)
        
        # 역방향 테스트
        print("Reverse...")
        reverse(pwm, 50)  # 50% 속도
        time.sleep(2)
        
        # 정지
        print("Stop...")
        stop(pwm)
        
    except KeyboardInterrupt:
        print("Program stopped by user")
    
    finally:
        pwm.stop()
        GPIO.cleanup()
        print("GPIO cleanup completed")

if __name__ == '__main__':
    main()