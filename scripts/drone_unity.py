#!/usr/bin/env python3
from pickle import NONE
import rospy
from geometry_msgs.msg import PoseStamped
from px4_control.srv import UnityGetState, UnityGetStateResponse, UnitySetState, UnitySetStateRequest, UnitySetStateResponse
import std_srvs.srv

from math import *
import numpy as np
from drone import Drone

class UnityDroneController:
    
    drone_state = UnityGetStateResponse.ONGROUND
    mode_state = UnitySetStateRequest.LAND

    def __init__(self) -> None:
        rospy.init_node('drone_control', anonymous=True)
        rospy.Service('unity/get_state', UnityGetState, self.handle_drone_state)
        rospy.Service('arm_active', std_srvs.srv.Trigger, self.handle_arm_state)
        rospy.Service('unity/set_state', UnitySetState, self.handle_drone_mode)
        rospy.Subscriber('unity/set_pose', PoseStamped, self.set_pose_callback)
        self.drone = Drone()
        self.states = {
            UnitySetStateRequest.TAKEOFF: self.__takeoff,
            UnitySetStateRequest.HOVER: self.__hover,
            UnitySetStateRequest.POSCTL: self.__position_control,
            UnitySetStateRequest.LAND: self.__land
        }
        self.set_point = None # hover set_point
        self.pose = None # movto pose
        self.control_provider()

    def handle_arm_state(self, req):
        if self.drone_state == UnityGetStateResponse.INAIR and \
            (self.mode_state == UnitySetStateRequest.HOVER or self.mode_state == UnitySetStateRequest.POSCTL):
            return std_srvs.srv.TriggerResponse(True, "Drone Ready")
        else:
            return std_srvs.srv.TriggerResponse(False, "Drone Busy")

    def handle_drone_state(self, req):
        return self.drone_state

    def handle_drone_mode(self, req):
        self.mode_state = req.set_state
        return UnitySetStateResponse(True)

    def set_pose_callback(self, data):
        self.pose = data

    def control_provider(self):
        prev_state = self.mode_state
        while not rospy.is_shutdown():
            if self.mode_state != prev_state:
                rospy.loginfo("[SM] Mode changed to %s", self.mode_state)
                prev_state = self.mode_state
            self.states[self.mode_state]()
            self.drone.rate.sleep()

    def __takeoff(self):
        if self.drone_state == UnityGetStateResponse.ONGROUND:
            self.drone.arm()
            self.drone.takeoff(1.0)
            self.drone_state = UnityGetStateResponse.INAIR
            self.mode_state = UnitySetStateRequest.HOVER

    def __hover(self):
        if self.set_point is None: 
            self.set_point = self.drone.pose
        self.drone.publish_setpoint(self.set_point, self.drone.yaw)
    
    def __position_control(self):
        if self.pose is not None:
            self.set_point = None
        else:
            sp = self.drone.pose
            self.pose = self.drone.get_setpoint(sp[0], sp[1], sp[2])
        self.drone.publish_pose(self.pose)

    def __land(self):
        if self.drone_state == UnityGetStateResponse.INAIR:
            self.drone.land()
            self.drone_state = UnityGetStateResponse.ONGROUND
            self.mode_state = UnitySetStateRequest.LAND

if __name__ == '__main__':
    try:
        UnityDroneController()
    except rospy.ROSInterruptException:
        pass