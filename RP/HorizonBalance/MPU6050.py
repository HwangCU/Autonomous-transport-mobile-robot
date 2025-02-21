import RPi.GPIO as GPIO
import smbus
from time import sleep


#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT = 0x3B
ACCEL_YOUT = 0x3D
ACCEL_ZOUT = 0x3F
GYRO_XOUT  = 0x43
GYRO_YOUT  = 0x45
GYRO_ZOUT  = 0x47


bus = smbus.SMBus(1) # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68 # MPU6050 device address


# Switch address name
switch_acc = {
    'x': ACCEL_XOUT,
    'y': ACCEL_YOUT,
    'z': ACCEL_ZOUT,
}

switch_gyro = {
    'x': GYRO_XOUT,
    'y': GYRO_YOUT,
    'z': GYRO_ZOUT,
}

# backup data when fail to read register data
pre_byte_data = {
    ACCEL_XOUT: 0,
    ACCEL_YOUT: 0,
    ACCEL_ZOUT: 0,
    GYRO_XOUT: 0,
    GYRO_YOUT: 0,
    GYRO_ZOUT: 0,
}

######################################################
######################################################

logger_prefix = "[MPU6050]"

prev_yaw = 90  # 초기값 90도

def safe_MPU_Init():
    try:
        #write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
        #Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
        #Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)
        #Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
        #Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)
        print(f"{logger_prefix} Initialization complete")
    except OSError as e:
        print(f"{logger_prefix} Initialization Error: {e}")

def safe_read_byte_data(addr):
    try:
        byte_data = bus.read_byte_data(Device_Address, addr)
        pre_byte_data[addr] = byte_data
        return byte_data
    except OSError as e:
        print(f"{logger_prefix} I2C Read Error at register {addr}: {e}")
        return pre_byte_data[addr]

def safe_read_raw_data(addr):
    try:
        #Accelero and Gyro value are 16-bit
        high = safe_read_byte_data(addr)
        sleep(0.01)
        low = safe_read_byte_data(addr+1)
        sleep(0.01)

        #concatenate higher and lower value
        value = ((high << 8) | low)

        #to get signed value from mpu6050
        if(value > 32768):
            value = value - 65536
        return value
    except OSError as e:
        print(f"I2C Read Error: {e}")

def safe_readAcc(axis: str) -> float:
    #Read Accelerometer raw value
    try:
        acc_ = safe_read_raw_data(switch_acc[axis])
    except KeyError as err:
        print("Not included in x, y, or z.", err)
    A_ = acc_/16384.0

    return A_

def safe_readYaw(axis: str) -> int:
    global prev_yaw
    try:
        acc_value = safe_readAcc(axis)
        
        in_min = -1
        in_max = 1
        out_min = 0
        out_max = 180
        value = (acc_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        value = int(value)
        
        prev_yaw = value
        return value
    except Exception as e:
        print(f"{logger_prefix} Error reading yaw: {e}")
        return prev_yaw

######################################################
######################################################
