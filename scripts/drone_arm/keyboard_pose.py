#!/usr/bin/env python3
import getch
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import *

def keys():
    pub = rospy.Publisher('unity/set_pose',PoseStamped,queue_size=1)
    rospy.init_node('keypress',anonymous=True)
    rate = rospy.Rate(10)
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1
    roll = 0
    pitch = 0
    yaw = 3.1415279/2
    while not rospy.is_shutdown():
        k=ord(getch.getch())
        #if ((k>=65)&(k<=68)|(k==115)|(k==113)|(k==97)):# to filter only the up , dowm ,left , right key /// this line can be removed or more key can be added to this
        if k == 119:
            pitch += 0.1
        elif k == 115:
            pitch -= 0.1
        elif k == 97:
            roll += 0.1
        elif k == 100:
            roll -= 0.1
        elif k == 27:
            rospy.signal_shutdown("EXIT")
        elif k == 113:
            yaw += 0.1
        elif k == 101:
            yaw -= 0.1
        elif k == 105:
            pose.pose.position.x += 0.1
        elif k == 107:
            pose.pose.position.x -= 0.1
        elif k == 106:
            pose.pose.position.y += 0.1
        elif k == 108:
            pose.pose.position.y -= 0.1
        elif k == 122:
            pose.pose.position.z += 0.1
        elif k == 120:
            pose.pose.position.z -= 0.1    
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.header.stamp = rospy.Time.now()
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        print(pose)
        pub.publish(pose)#to publish
        rate.sleep()

#s=115,e=101,g=103,b=98

if __name__=='__main__':
    try:
        keys()
    except rospy.ROSInterruptException:
        pass