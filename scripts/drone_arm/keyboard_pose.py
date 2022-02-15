#!/usr/bin/env python3
import getch
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from px4_control.srv import UnitySetState, UnitySetStateRequest

def keys():
    pub = rospy.Publisher('unity/set_pose',PoseStamped,queue_size=1)
    rospy.init_node('keypress',anonymous=True)
    chmode = rospy.ServiceProxy('unity/set_state', UnitySetState)
    rate = rospy.Rate(10)
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1
    roll = 0
    pitch = 0
    yaw = 3.1415279/2
    rospy.logwarn("\nControls:\n ESC - exit\n W,A,S,D - change position along X,Y\n Q,E - yaw\n Z - up, X - down\n 1 - land, 2 - takeoff, 3 - hover, 4 - position control")
    while not rospy.is_shutdown():
        k=ord(getch.getch())
        
        if k == 119: #W
            pose.pose.position.x += 0.1
        elif k == 115: #S
            pose.pose.position.x -= 0.1
        elif k == 97: #A
            pose.pose.position.y += 0.1
        elif k == 100: #D
            pose.pose.position.y -= 0.1
        elif k == 27: #ESC
            rospy.signal_shutdown("EXIT")
        elif k == 113: #Q
            yaw += 0.1
        elif k == 101: #E
            yaw -= 0.1
        elif k == 122: #X
            pose.pose.position.z += 0.1
        elif k == 120: #Z
            pose.pose.position.z -= 0.1   
        elif k == 49: #1
            chmode(UnitySetStateRequest.LAND)
        elif k == 50: #2
            chmode(UnitySetStateRequest.TAKEOFF)
        elif k == 51: #3
            chmode(UnitySetStateRequest.HOVER)
        elif k == 52: #4
            chmode(UnitySetStateRequest.POSCTL)
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.header.stamp = rospy.Time.now()
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pub.publish(pose)#to publish
        rate.sleep()

#s=115,e=101,g=103,b=98

if __name__=='__main__':
    try:
        keys()
    except rospy.ROSInterruptException:
        pass