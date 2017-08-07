#!/usr/bin/env python

################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : loop_skill.py
# Authors : Ludovic DELVAL
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Ludovic DELVAL <ludovic.delval.external@airbus.com>
#
#
################################################################################

from ssm_core import ssm_state
import math
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D


class CameraUR10_Trigger(ssm_state.ssmState):
    
    def __init__(self):
        ssm_state.ssmState.__init__(self, io_keys = ['correction_cam'], outcomes=['fail', 'success', 'approach'])
        
        self.trigger_pub   = None
        self.Pose2D_sub = None
        self.pose_camera = Pose2D()
        
    
    def camera_pose_cb(self, msg):
            self.pose_camera = msg
        
    def scan_hole_position(self, ud):
        cpt = 0
        r = rospy.Rate(100)
        self.pose_camera.theta = 0
        rospy.sleep(0.1) ##Be sure that the camera is stable !
        self.trigger_pub.publish()
       
        while ((not self.preempt_requested()) and (int(self.pose_camera.theta) == 0)):
            if (cpt > 1000): ##10seconds
                rospy.logerr("[Camera UR10] : Execution Error !")
                return "preempt"
            else:
                cpt = cpt + 1
                r.sleep()
        if(int(self.pose_camera.theta)==-1):
            rospy.logerr("[Camera UR10] : Detection failed !")
            return "fail"
        
        elif(int(self.pose_camera.theta)==1):
            ud.correction_cam[0] = ud.correction_cam[0] + self.pose_camera.x/1000.0
            ud.correction_cam[1] = ud.correction_cam[1] + self.pose_camera.y/1000.0
            dist_ = math.sqrt(self.pose_camera.x*self.pose_camera.x + self.pose_camera.y * self.pose_camera.y)
            if(dist_ > 9.0):
                return "approach"
            else:
                return "success"
        else:
            rospy.logerr("[Camera UR10] : Detection error !")
            return "preempt"
        
    def execution(self, ud):
        if(ud.correction_cam == None):
            ud.correction_cam = [0.0,0.0]
        self.trigger_pub   = rospy.Publisher("/camera_ur10/trigger",Empty,queue_size=1)
        self.Pose2D_sub = rospy.Subscriber("/camera_ur10/pose",Pose2D, self.camera_pose_cb)
        outcome = self.scan_hole_position(ud)
        self.trigger_pub = None
        self.Pose2D_sub = None
        return outcome


class ComputeOffset(ssm_state.ssmState):
    
    def __init__(self):
        ssm_state.ssmState.__init__(self, io_keys = ['correction_cam','seekCam'], outcomes=['fail', 'success'])

    def execution(self, ud):
        if(int(ud.seekCam)==0):
            ud.seekCam=1
            ud.correction_cam[0] = float(ud.correction_cam[0]) + 0.025
            ud.correction_cam[1] = float(ud.correction_cam[1]) + 0.025
            return "success"
        elif(int(ud.seekCam)==1):
            ud.seekCam=2
            ud.correction_cam[0] = float(ud.correction_cam[0]) - 0.05
            return "success"
        elif(int(ud.seekCam)==2):
            ud.seekCam=3
            ud.correction_cam[1] = float(ud.correction_cam[1]) - 0.05
            return "success"
        elif(int(ud.seekCam)==3):
            ud.seekCam = 4
            ud.correction_cam[0] = float(ud.correction_cam[0]) + 0.05
            return "success"
        else:
            ud.seekCam = -1
            ud.correction_cam[0] = float(ud.correction_cam[0]) - 0.025
            ud.correction_cam[1] = float(ud.correction_cam[1]) + 0.025
            return "fail"
               
        
