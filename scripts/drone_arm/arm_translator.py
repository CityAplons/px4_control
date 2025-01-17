#!/usr/bin/env python3

from math import fabs
import rospy
import serial 
from std_msgs.msg import String
from std_srvs.srv import Trigger

# set serial parameters
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_RATE = 115200
is_active = rospy.ServiceProxy('/arm_active', Trigger)
isArmInitialized = False

def callback(data):
    global isArmInitialized
    b = bytes(data.data, 'utf-8')

    #ser.write(b)
    if is_active().success:
        isArmInitialized = True
        ser.write(b)
    else:
        isArmInitialized = False

if __name__ == '__main__':
    ser = serial.Serial(SERIAL_PORT, SERIAL_RATE)

    rospy.init_node('arm_control', anonymous=True)

    hz = 20
    rate = rospy.Rate(hz)

    pub = rospy.Publisher('unity/arm_out', String, queue_size=10)
    rospy.Subscriber("unity/arm_in", String, callback)

    calledOnce = False
    rate.sleep()
    while not rospy.is_shutdown():
        
        if not isArmInitialized and not calledOnce:
            calledOnce = True
            ser.write(bytes("m -180.0 -1.0 -90.0 -90.0 65.0\r\n", "utf-8"))
        if isArmInitialized:
            calledOnce = False

        # get data from serial
        reading = ser.readline().decode('utf-8')
        pub.publish(reading)
        rate.sleep()


