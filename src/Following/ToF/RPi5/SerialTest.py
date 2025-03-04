import serial
import matplotlib.pyplot as plt

# UART3 (ttyAMA3) 설정
ser = serial.Serial('/dev/ttyAMA3', baudrate=9600, timeout=1, bytesize=serial.EIGHTBITS)


serial_val = "origin"
# serial_val = "kalman"

i = 0
i_val = []
kalman = []
origin = []

while i < 100:
    if ser.in_waiting > 0:
        data = ser.readline(ser.in_waiting)
        i += 1
        
        decoded_data = data.decode('ascii', errors="ignore")
        print(decoded_data)

        if serial_val == "kalman":
            #kalman datas
            kalman.append(decoded_data)

        elif serial_val == "origin":
            # #origin datas
            origin.append(decoded_data)
        

plt.figure()
plt.xlabel('Times')
plt.ylabel('Distance(m)')

# ran = range(min(len(origin),len(kalman)))
if serial_val == "kalman":
    plt.title('Kalman')
    ran = range(1, len(kalman)+1)
    plt.plot(ran,kalman,'bo',ran,kalman,'r--')
    plt.savefig('origin.png')


elif serial_val == "origin":
    plt.title('Origin')
    ran = range(1, len(origin)+1)
    plt.plot(ran,origin,'bo',ran,origin,'r--')
    plt.savefig('origin.png')