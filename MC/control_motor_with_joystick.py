from evdev import InputDevice, categorize, ecodes
import Jetson.GPIO as GPIO
import time


# 핀 설정
left_pwm1 = 33
left_dir1 = 26
right_pwm2 = 32
right_dir2 = 23

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
    GPIO.setup(left_dir1, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(right_dir2, GPIO.OUT, initial=GPIO.LOW)

    # PWM 객체 생성 - 다른 주파수 사용
    GPIO.setup(left_pwm1, GPIO.OUT, initial=GPIO.HIGH)
    left_pwm = GPIO.PWM(left_pwm1, 10000)  # 1KHz
    left_pwm.start(0)

    time.sleep(0.1)
    GPIO.setup(right_pwm2, GPIO.OUT, initial=GPIO.HIGH)
    right_pwm = GPIO.PWM(right_pwm2, 10000)   # 1KHz
    right_pwm.start(0)

    # PWM 시작

    return left_pwm, right_pwm

prev_pwm = 0
def p_control(motor, value):
    p = 0.1
    while prev_pwm < value:
        diff = value - prev_pwm
        prev_pwm += diff * p
        motor.ChangeDutyCycle(stable_pwm)
        time.sleep(0.1)



def main():
    global forward, control_left_pwm, control_right_pwm, stable_pwm, prev_pwm
    try:
        lpwm, rpwm = motor_setup()
        print("Starting controlling...")
        
 
        for event in joystick.read_loop():
            if event.type == ecodes.EV_KEY:
                if event.code == 308:
                    print("Forward...")
                    
                    GPIO.output(left_dir1, GPIO.LOW)
                    GPIO.output(right_dir2, GPIO.LOW)
                    if event.value:
                        forward = 1
                        stable_pwm = 20
                    else:
                        forward = 0
                        stable_pwm = 0

                elif event.code == 304:
                    print("Backward...")

                    GPIO.output(left_dir1, GPIO.HIGH)
                    GPIO.output(right_dir2, GPIO.HIGH)
                    if event.value:
                        forward = -1
                        stable_pwm = 20
                    else:
                        forward = 0
                        stable_pwm = 0
                p_control(rpwm,stable_pwm)
                p_control(lpwm,stable_pwm)



            elif event.type == ecodes.EV_ABS:
                if event.code == 1:
                    if event.value == -1:
                        forward = 0
                        pwm = 0

                    else:
                        # pwm = int(abs(event.value) /327)
                        pwm = int(abs(event.value) /468)


                        if event.value < -1:
                            print(f"Forward... pwm:{pwm}")

                            GPIO.output(left_dir1, GPIO.LOW)
                            GPIO.output(right_dir2, GPIO.LOW)
                            forward = 1

                        elif event.value > 0 :
                            print(f"Backward... pwm:{pwm}")

                            GPIO.output(left_dir1, GPIO.HIGH)
                            GPIO.output(right_dir2, GPIO.HIGH)
                            forward = -1
                    control_left_pwm = pwm
                    control_right_pwm = pwm


                if forward != 0 and event.code == 3:
                    if event.value == -1:
                        control_left_pwm = pwm
                        control_right_pwm = pwm
                        
                    else:
                        # low_pwm = pwm - (pwm / 100 * int(abs(event.value) /327)) / 2
                        low_pwm = pwm - (pwm / 100 * int(abs(event.value) /468)) / 2

                        if event.value < -1:
                            print(f"    Left:... lpwm: {control_left_pwm}, right: {control_right_pwm}")
                            control_left_pwm = low_pwm

                        elif event.value > 0 :
                            print(f"    Right:... lpwm: {control_left_pwm}, right: {control_right_pwm}")
                            control_right_pwm = low_pwm
                p_control(lpwm,control_left_pwm)
                p_control(rpwm, control_right_pwm)

    except KeyboardInterrupt:
        print("Program stopped by user")
    
    finally:
        rpwm.stop()
        lpwm.stop()
        GPIO.cleanup()
        print("GPIO cleanup completed")

if __name__ == '__main__':
    main()



