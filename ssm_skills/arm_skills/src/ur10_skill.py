#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : read_xml.py
# Authors : Ludovic DELVAL
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Ludovic DELVAL <ludovic.delval.external@airbus.com>
#
#
################################################################################

import rospy
import numpy
import tf
from ssm_core import ssm_state
from ur10_driver.msg  import UR10Cartesian
from std_msgs.msg import Int16, Empty

def Format44quat(pose = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]):
    px_ = pose[0]
    py_ = pose[1]
    pz_ = pose[2]
    qx_ = pose[3]
    qy_ = pose[4]
    qz_ = pose[5]
    qw_ = pose[6]  
        
    Mat = numpy.eye(4) ##Init
    Mat_ = tf.transformations.quaternion_matrix([qx_, qy_, qz_, qw_])
    Mat[0][0] = Mat_[0][0]
    Mat[1][0] = Mat_[1][0]
    Mat[2][0] = Mat_[2][0]
    Mat[0][1] = Mat_[0][1]
    Mat[1][1] = Mat_[1][1]
    Mat[2][1] = Mat_[2][1]
    Mat[0][2] = Mat_[0][2]
    Mat[1][2] = Mat_[1][2]
    Mat[2][2] = Mat_[2][2]
    Mat[0][3] = px_
    Mat[1][3] = py_
    Mat[2][3] = pz_
    
    return Mat

class UR10CartesianMotion( ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self, io_keys = ['target_point' , 'tool' , 'CartesianSpeed' , 'CartesianAcc' , 
                                                     'useCorrection' , 'correction_cam' , 'useOffset' , 
                                                     'offset_x' , 'offset_y' , 'offset_z'], 
                                          outcomes = ['succeeded'])
        
        self.target_pt_pub   = rospy.Publisher("/ur10/target_point",UR10Cartesian,queue_size=1)
        self.robot_state_sub = None
        self.robot_state     = 0
        
    
    def robot_state_cb(self, msg):
            self.robot_state = msg.data
        
    def execution(self, ud):  
        self.robot_state_sub = rospy.Subscriber("/ur10/state",Int16, self.robot_state_cb)  
        cpt = 0
        r = rospy.Rate(100)  
        target_pt =  UR10Cartesian()

        target_pt.header = ud.target_point.header
        target_pt.pose.position.x = ud.target_point.pose.position.x
        target_pt.pose.position.y = ud.target_point.pose.position.y
        target_pt.pose.position.z = ud.target_point.pose.position.z
        target_pt.pose.orientation.x = ud.target_point.pose.orientation.x
        target_pt.pose.orientation.y = ud.target_point.pose.orientation.y
        target_pt.pose.orientation.z = ud.target_point.pose.orientation.z
        target_pt.pose.orientation.w = ud.target_point.pose.orientation.w
        target_pt.tool         = int(ud.tool)
        target_pt.speed        = float(ud.CartesianSpeed)
        target_pt.acceleration = float(ud.CartesianAcc)
        if(target_pt == None):
            rospy.logerr("[UR10 Skill] : Error in target pose.")
            return 'preempt'
        ##Correct target PT with offset
        if (int(ud.useCorrection)):
            if(ud.correction_cam == None):
                ud.correction_cam = [0.0, 0.0]
            pose = [target_pt.pose.position.x, target_pt.pose.position.y, target_pt.pose.position.z, 
                    target_pt.pose.orientation.x, target_pt.pose.orientation.y, target_pt.pose.orientation.z, target_pt.pose.orientation.w]
            mat     = Format44quat(pose)
            mat2    = [ud.correction_cam[1], ud.correction_cam[0], 0.0, 1.0]
            result = numpy.dot(mat,mat2)

            target_pt.pose.position.x = result[0]
            target_pt.pose.position.y = result[1]
            ud.useCorrection = 0
        if(int(ud.useOffset)):
            target_pt.pose.position.x = target_pt.pose.position.x + float(ud.offset_x)
            target_pt.pose.position.y = target_pt.pose.position.y + float(ud.offset_y)
            target_pt.pose.position.z = target_pt.pose.position.z + float(ud.offset_z)
            ud.offset_x = 0.000
            ud.offset_y = 0.000
            ud.offset_z = 0.000
            ud.useOffset = 0
        self.target_pt_pub.publish(target_pt)
        while ((not self.preempt_requested()) and (not self.robot_state == 1)):
            if ((cpt > 1000) or (self.robot_state == -1)): ##10seconds
                rospy.logerr("[UR10 Skill] : Execution Error !")
                self.execution = False
                #return 'preempt'
                return 'preempt'
            else:
                cpt = cpt + 1
                r.sleep()
                
        rospy.loginfo("[UR10 Skill] : Moving !")
        
        cpt = 0
        while ((not self.preempt_requested()) and (not self.robot_state == 0)):
            if ((cpt > 60000)  or (self.robot_state == -1)): ##60seconds
                rospy.logerr("[UR10 Skill] : Movement Error")
                self.execution = False
                return 'preempt'
            else:
                 cpt = cpt + 1
                 r.sleep()
        
        rospy.loginfo("[UR10 Skill] : Clearing Trajectory Buffer!")
        return 'succeeded'
    
class UR10TriggerAction(ssm_state.ssmState):
        
    
    def __init__(self):
        ssm_state.ssmState.__init__(self, io_keys = [] , outcomes = ['failed' , 'succeeded'])
        self.actionner_pub   = rospy.Publisher("/ur10/actionner",Empty,queue_size=1)
        self.robot_state_sub = None
        self.robot_state     = 0
        
    
    def robot_state_cb(self, msg):
            self.robot_state = msg.data
        
    def execution(self, ud): 
        self.robot_state_sub = rospy.Subscriber("/ur10/state",Int16, self.robot_state_cb)   
        cpt = 0
        r = rospy.Rate(100)    
        self.actionner_pub.publish()
        while ((not self.preempt_requested()) and (not self.robot_state == 3)):
            if ((cpt > 1000) or (self.robot_state == -1)): ##10seconds
                rospy.logerr("[UR10 Skill] : Execution Error !")
                return 'preempt'
            else:
                cpt = cpt + 1
                r.sleep()
                
        rospy.loginfo("[UR10 Skill] : Actionning !")
        
        cpt = 0
        while ((not self.preempt_requested()) and (not self.robot_state == 4)):
            if ((cpt > 1000)  or (self.robot_state == -1)): ##60seconds
                rospy.logerr("[UR10 Skill] : Actionning error !")
                return 'preempt'
            elif(self.robot_state == -2):
                rospy.logerr("[UR10 Skill] : Actionning failed !")
                return 'failed'
            else:
                 cpt = cpt + 1
                 r.sleep()
        return 'succeeded'
