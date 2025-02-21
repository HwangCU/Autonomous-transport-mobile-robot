import RPi.GPIO as GPIO
from time import sleep

logger_prefix = "[SG90]"

servoPin = 4			# Servo Signal Pin Num
SERVO_MAX_DUTY = 12		# Servo Max Angle(180deg) Duty time
SERVO_MIN_DUTY = 2		# Servo Min Angle(0deg) Duty time

try:
    GPIO.setmode(GPIO.BCM)				# GPIO Pin name setting
    GPIO.setup(servoPin, GPIO.OUT)		# Set servoPin to output
    print(f"{logger_prefix} GPIO setup complete")

    servo = GPIO.PWM(servoPin, 50)		# PWM Period setting
    servo.start(0)						# Start PWM (duty=0)
except Exception as e:
    print(f"{logger_prefix} Error initializing servo: {e}")

level_angle = 80.0        # Horizontal angle

def setServoPos(degree: float) -> None:
    try:
        # Angle limit
        # if degree > level_angle + 38:
        #     degree = level_angle + 38
        # elif degree < level_angle - 38:
        #     degree = level_angle - 38
        
        # convert degree to duty
        duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
        
        # Input 'duty' to PWM
        servo.ChangeDutyCycle(duty)
    except Exception as e:
        print(f"{logger_prefix} Error setting servo position: {e}")

def movServo(degree: int) -> None:
    try:
        MovTerm = 0.24
        setServoPos(degree)
        sleep(MovTerm)
    except Exception as e:
        print(f"{logger_prefix} Error in movServo: {e}")

def startServo():
    try:
        print(f"{logger_prefix} Initializing servo to level angle {level_angle}")
        setServoPos(level_angle)
        sleep(1)
    except Exception as e:
        print(f"{logger_prefix} Error in startServo: {e}")

def endServo():
    try:
        # Zero DutyCycle 
        servo.ChangeDutyCycle(0)
        sleep(0.1)
        # PWM Stop
        servo.stop()
        # Initialize GPIO
        GPIO.cleanup()
        print(f"{logger_prefix} Servo stopped and GPIO cleaned up")
    except Exception as e:
        print(f"{logger_prefix} Error in endServo: {e}")
        try:
            GPIO.cleanup()
        except:
            pass
