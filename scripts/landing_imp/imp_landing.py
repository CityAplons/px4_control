#!/usr/bin/env python

import rospy
import mavros
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State 
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import *
from math import *
import numpy as np
import argparse
import sys
import impedance_modeles
import time

# callback method for state sub
current_state = State() 
offb_set_mode = SetMode
global sp
sp = PoseStamped()


def state_cb(state):
    global current_state
    current_state = state

# callback method for local pos sub
def locpos_cb(locpos):
    global lp
    lp = locpos

def vel_cb(data):
    global drone_vel
    drone_vel = np.array([data.twist.linear.x, data.twist.linear.y, data.twist.linear.z])

local_pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, locpos_cb)
vel_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, vel_cb)
local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
state_sub = rospy.Subscriber('/mavros/state', State, state_cb)
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)

def takeoff(height):
    rospy.loginfo("Takeoff")
    sp.pose.position.z = 0
    while sp.pose.position.z < height:
        sp.header.stamp = rospy.Time.now()
        sp.pose.position.z += 0.05
        local_pos_pub.publish(sp)
        rate.sleep()

def landing():
    rospy.loginfo("Landing...")
    sp.pose.position.z = lp.pose.position.z

    imp_pose_prev = np.array( [0,0,0] )
    imp_vel_prev = np.array( [0,0,0] )
    imp_time_prev = time.time()

    while sp.pose.position.z > -0.5:
        sp.header.stamp = rospy.Time.now()
        sp.pose.position.z -= 0.01
        """
        TODO: correct landing pose with impedance model
        """
        # ipedance terms calculation
        imp_pose, imp_vel, imp_time_prev = impedance_modeles.velocity_impedance(drone_vel, imp_pose_prev, imp_vel_prev, imp_time_prev)
        imp_pose_prev = imp_pose
        imp_vel_prev = imp_vel
        
        # correct pose with impedance model
        sp.pose.position.z += imp_pose[2]
        print drone_vel[2]

        local_pos_pub.publish(sp)
        rate.sleep()


def position_control(h_des, t_hold):
    rospy.init_node('des_location', anonymous=True)
    prev_state = current_state
    global rate, x_home, y_home
    rate = rospy.Rate(20.0) # MUST be more then 2Hz

    x_home = lp.pose.position.x
    y_home = lp.pose.position.y
    # send a few setpoints before starting
    sp.pose.position.x = lp.pose.position.x
    sp.pose.position.y = lp.pose.position.y
    sp.pose.position.z = -1
    sp.pose.orientation = lp.pose.orientation
    
    for i in range(10):
        local_pos_pub.publish(sp)
        rate.sleep()
    
    # wait for FCU connection
    # print "FCU connection..."
    # while not current_state.connected:
    #     rate.sleep()

    last_request = rospy.get_time()
    while not rospy.is_shutdown():
        now = rospy.get_time()
        if current_state.mode != "OFFBOARD" and (now - last_request > 2.):
            set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            last_request = now 
        else:
            if not current_state.armed and (now - last_request > 2.):
               arming_client(True)
               last_request = now 

        # older versions of PX4 always return success==True, so better to check Status instead
        if prev_state.armed != current_state.armed:
            rospy.loginfo("Vehicle armed: %r" % current_state.armed)

        if prev_state.mode != current_state.mode: 
            rospy.loginfo("Current mode: %s" % current_state.mode)
        prev_state = current_state

        if current_state.armed:
            break
        # Update timestamp and publish sp 
        sp.header.stamp = rospy.Time.now()
        local_pos_pub.publish(sp)
        rate.sleep()

    takeoff(height=h_des)
    
    # mission
    rospy.loginfo('Position holding...')
    mission_start = rospy.get_time()
    sp.pose.position.x = x_home
    sp.pose.position.y = y_home
    sp.pose.position.z = h_des
    while not rospy.is_shutdown():
        mission_end = rospy.get_time()
        if mission_end - mission_start > t_hold and t_hold > 0:
            break
        # Update timestamp and publish sp 
        sp.header.stamp = rospy.Time.now()
        local_pos_pub.publish(sp)
        rate.sleep()
    
    landing()

    # disarming
    while current_state.armed or current_state.mode == "OFFBOARD":
        now = rospy.get_time()
        if current_state.armed:
            arming_client(False)
        if current_state.mode == "OFFBOARD":
            set_mode_client(base_mode=0, custom_mode="MANUAL")
            last_request = now
        rate.sleep()

if __name__ == '__main__':
    try:
        parser = argparse.ArgumentParser(description="Command line tool for setting desired altitude and time of holding it.")
        parser.add_argument('-alt', '--altitude', type=float, nargs=1, help="Desired takeoff altitude")
        parser.add_argument('-t', '--holdtime', type=float, nargs=1, help="Altitude holding time")
        args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
        try:
            h_des = args.altitude[0]
            t_hold = args.holdtime[0]
        except:
            h_des = 1.0; t_hold = 10.0
            print("Desired altitude: 1 m; holding time: 2 sec")
        position_control(h_des, t_hold)
    except rospy.ROSInterruptException:
        pass