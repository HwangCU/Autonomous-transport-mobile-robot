import RPi.GPIO as GPIO
from time import sleep

servoPin = 4			# Servo Signal Pin Num
SERVO_MAX_DUTY = 12		# Servo Max Angle(180deg) Duty time
SERVO_MIN_DUTY = 2		# Servo Min Angle(0deg) Duty time

GPIO.setmode(GPIO.BCM)				# GPIO Pin name setting
GPIO.setup(servoPin, GPIO.OUT)		# Set servoPin to output

servo = GPIO.PWM(servoPin, 50)		# PWM Period setting
servo.start(0)						# Start PWM (duty=0)

level_angle = 80.0        # Horizontal angle

def setServoPos(degree: float) -> None:
    # Angle limit
    # if degree > level_angle + 38:
    #     degree = level_angle + 38
    # elif degree < level_angle - 38:
    #     degree = level_angle - 38
    
    # convert degree to duty
    duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
    
    # Input 'duty' to PWM
    servo.ChangeDutyCycle(duty)

def movServo(degree: int) -> None:
    MovTerm = 0.24
    setServoPos(degree)
    sleep(MovTerm)

def startServo():
    print("Initialize Level Angle")
    setServoPos(level_angle)
    sleep(1)

def endServo():
    print("End Motor and clean up GPIO")
    # Zero DutyCycle 
    servo.ChangeDutyCycle(0)
    sleep(0.1)
    # PWM Stop
    servo.stop()
    # Initialize GPIO
    GPIO.cleanup()
