import RPi.GPIO as GPIO
import time

print("GPIO Model:", GPIO.model)
output_pins = {
    'JETSON_XAVIER': 18,
    'JETSON_NANO': 33,
    'JETSON_NX': 33,
    'CLARA_AGX_XAVIER': 18,
    'JETSON_TX2_NX': 32,
    'JETSON_ORIN': 18,
    'JETSON_ORIN_NX': 33,
    'JETSON_ORIN_NANO': 33
}

output_pin = output_pins.get(GPIO.model, None)

if output_pin is None:
    raise Exception('PWM not supported on this board')

ENA_pin = 33  # PWM pin for Jetson Orin Nano
IN1_pin = 31  # Direction control 1
IN2_pin = 29  # Direction control 2

def main():
    GPIO.setmode(GPIO.BOARD)
    mode = GPIO.getmode()
    print(f"GPIO mode: {mode}")
    GPIO.setwarnings(False)
    
    GPIO.setup(ENA_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN1_pin, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(IN2_pin, GPIO.OUT, initial=GPIO.LOW)
    try:
        while True:
            pass
        
    except KeyboardInterrupt:
        print("Program stopped by user")
    
    finally: 
        GPIO.cleanup()
        print("GPIO cleanup completed")

if __name__ == '__main__':
    main()