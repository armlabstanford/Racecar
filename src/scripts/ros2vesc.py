import serial,time
from pyvesc import VESC
import rospy
from std_msgs.msg import Float32

throttle = 0.0

def throttle_callback(data):
    global throttle
    throttle = (data.data/255)*0.3

serial_port = '/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00'

vesc = VESC(serial_port)

rospy.init_node('VESCSerial', anonymous=True)
rospy.Subscriber("throttle", Float32, throttle_callback)

while True:
    print(throttle)
    vesc.set_duty_cycle(throttle*0.5)
    time.sleep(0.02)