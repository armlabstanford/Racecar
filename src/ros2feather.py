import serial
import time
import rospy
from std_msgs.msg import Float32

def steer_callback(data):
    global steer_angle
    steer_angle = (data.data/255-0.5)*110+85

HZ = 50
rospy.init_node('FeatherSerial', anonymous=True)
rospy.Subscriber("steer", Float32, steer_callback)
steer_angle = 85.0

# Open the serial port.
port = '/dev/serial/by-id/usb-Adafruit_Industries_LLC_Feather_M4_CAN_903C8AA83247395320202037383611FF-if00'
ser = serial.Serial(port, 115200, timeout=0.1)
print('Connected')
time.sleep(2)

# Write a command to the Feather board.
ser.write(b"90\r")
t2 = time.time()
# ser.flush()
print('Sent command:')

# Read the response.
while True:
    t1 = time.time()
    print('\n[============]')
    response = ser.readline().decode()
    print("RS:",response)
    while ser.in_waiting:
        response = ser.readline().decode()
        if response[0] == "R":
            print('Response:', response)
            # print(time.time()-t2)
            # print('dd',time.time()-t1)
    
    cmd = (str(int(steer_angle))+"\r").encode()
    print(cmd)
    ser.write(cmd)
    ser.flush()
    print('Done Writing')

    # t2 = time.time()
    # time.sleep(max(0,1/HZ-(t2-t1)-0.00006))
    # t3 = time.time()
    # print('Time:',t3-t1)
    time.sleep(0.02)

# Close the serial port.
ser.close()