from evdev import InputDevice, categorize, ecodes
import Jetson.GPIO as GPIO
import time


# 핀 설정
right_pwm1 = 33
right_dir1 = 26
left_pwm2 = 32
left_dir2 = 23

pwm = 0
stable_pwm = 20
control_left_pwm = 0
control_right_pwm = 0
forward = 0; #1: forward, 0: stop, -1: backward

device_path = "/dev/input/event5" 
joystick = InputDevice(device_path)

print(f"연결된 컨트롤러: {joystick.name}")


def motor_setup():
    # 기존의 GPIO 설정을 초기화
    GPIO.cleanup()
    
    # GPIO 모드 설정
    GPIO.setmode(GPIO.BOARD)
    
    # 핀 설정
    GPIO.setup(right_dir1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(left_dir2, GPIO.OUT, initial=GPIO.LOW)

    # PWM 객체 생성 - 다른 주파수 사용
    GPIO.setup(right_pwm1, GPIO.OUT, initial=GPIO.HIGH)
    right_pwm = GPIO.PWM(right_pwm1, 10000)  # 1KHz
    right_pwm.start(0)

    time.sleep(0.1)
    GPIO.setup(left_pwm2, GPIO.OUT, initial=GPIO.HIGH)
    left_pwm = GPIO.PWM(left_pwm2, 10000)   # 1KHz
    left_pwm.start(0)

    # PWM 시작

    return right_pwm, left_pwm


def main():
    global forward, control_left_pwm, control_right_pwm, stable_pwm
    try:
        rpwm, lpwm = motor_setup()
        print("Starting controlling...")
        
 
        for event in joystick.read_loop():
            if event.type == ecodes.EV_KEY:
                if event.code == 308:
                    print("Forward...")
                    GPIO.output(right_dir1, GPIO.HIGH)
                    GPIO.output(left_dir2, GPIO.HIGH)
                    if event.value:
                        forward = 1
                        stable_pwm = 20
                    else:
                        forward = 0
                        stable_pwm = 0

                elif event.code == 304:
                    print("Backward...")
                    GPIO.output(right_dir1, GPIO.LOW)
                    GPIO.output(left_dir2, GPIO.LOW)
                    if event.value:
                        forward = -1
                        stable_pwm = 20
                    else:
                        forward = 0
                        stable_pwm = 0
                rpwm.ChangeDutyCycle(stable_pwm)
                lpwm.ChangeDutyCycle(stable_pwm)


            elif event.type == ecodes.EV_ABS:
                if event.code == 1:
                    if event.value == -1:
                        forward = 0
                        pwm = 0

                    else:
                        pwm = int(abs(event.value) /327)

                        if event.value < -1:
                            print(f"Forward... pwm:{pwm}")

                            GPIO.output(right_dir1, GPIO.LOW)
                            GPIO.output(left_dir2, GPIO.LOW)
                            forward = 1

                        elif event.value > 0 :
                            print(f"Backward... pwm:{pwm}")

                            GPIO.output(right_dir1, GPIO.HIGH)
                            GPIO.output(left_dir2, GPIO.HIGH)
                            forward = -1
                    control_left_pwm = pwm
                    control_right_pwm = pwm


                if forward != 0 and event.code == 3:
                    if event.value == -1:
                        control_left_pwm = pwm
                        control_right_pwm = pwm
                        
                    else:
                        low_pwm = pwm - (pwm / 100 * int(abs(event.value) /327)) / 2
                        if event.value < -1:
                            print(f"    Left:... lpwm: {control_left_pwm}, right: {control_right_pwm}")
                            control_left_pwm = low_pwm

                        elif event.value > 0 :
                            print(f"    Right:... lpwm: {control_left_pwm}, right: {control_right_pwm}")
                            control_right_pwm = low_pwm
                rpwm.ChangeDutyCycle(control_left_pwm)
                lpwm.ChangeDutyCycle(control_right_pwm)

    except KeyboardInterrupt:
        print("Program stopped by user")
    
    finally:
        rpwm.stop()
        lpwm.stop()
        GPIO.cleanup()
        print("GPIO cleanup completed")

if __name__ == '__main__':
    main()




