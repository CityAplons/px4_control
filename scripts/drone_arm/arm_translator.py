#!/usr/bin/env python3

import rospy
import serial 
from std_msgs.msg import String

# set serial parameters
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_RATE = 115200
is_active = rospy.ServiceProxy('/arm_active_service')

# arm_active_service

def callback(data):
    b = bytes(data.data, 'utf-8')

    if is_active().success:
        ser.write(b)

if __name__ == '__main__':
    ser = serial.Serial(SERIAL_PORT, SERIAL_RATE)

    rospy.init_node('arm_control', anonymous=True)

    hz = 20
    rate = rospy.Rate(hz)

    pub = rospy.Publisher('unity/arm_out', String, queue_size=10)
    rospy.Subscriber("unity/arm_in", String, callback)

    while not rospy.is_shutdown():
        # get data from serial
        reading = ser.readline().decode('utf-8')
        pub.publish(reading)
        rate.sleep()


