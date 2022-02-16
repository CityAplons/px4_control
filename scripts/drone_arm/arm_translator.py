#!/usr/bin/env python3

from math import fabs
import rospy
import serial 
from std_msgs.msg import String

# set serial parameters
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_RATE = 115200
is_active = rospy.ServiceProxy('/arm_active_service')
isArmInitialized = False

def callback(data):
    global isArmInitialized
    b = bytes(data.data, 'utf-8')

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

    while not rospy.is_shutdown():
        
        if not isArmInitialized and not calledOnce:
            calledOnce = True
            # TODO: command to reset arm
            ser.write(bytes(""))
        if isArmInitialized:
            calledOnce = False

        # get data from serial
        reading = ser.readline().decode('utf-8')
        pub.publish(reading)
        rate.sleep()


