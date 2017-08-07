#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : states.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import rospy
import numpy
from ssm_core import ssm_state
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D, PoseStamped
from xml.etree import ElementTree as ET
from tf import TransformListener
from read_xml import *
from convertion_tools import *
from pyqt_agi_extend.QtAgiCore import get_pkg_dir_from_prefix

class WaitForTrackingInit(ssm_state.ssmState):
    
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes = ["success", "fail"])
        
        self.initTrigger_pub    = rospy.Publisher("/vp_tracker/triger_init",Empty, queue_size = 1)
        self.initTracking_sub   = None
        self.execution_              = False
        self.tracking_initialized   = False
        
    def initTracking_cb(self, msg):
		if(self.execution_ == True):
			self.tracking_initialized = True
        
    def execution(self, ud):
        rospy.loginfo("HERE")
        self.initTracking_sub   = rospy.Subscriber("/vp_tracker/tracking_initialized",Empty, self.initTracking_cb)
        self.tracking_initialized = False
        self.execution_ = True
        cpt = 0
        r = rospy.Rate(10)
        self.initTrigger_pub.publish()
        while ((self.tracking_initialized == False)):
            if self.preempt_requested():
                return "preempt"
            if ((cpt > 300)): ##30seconds                
                rospy.logerr("[Visp Tracker] : Initialisation Failed !")
                self.execution = False
                return "fail"
            else:
                cpt = cpt + 1
                r.sleep()
        rospy.loginfo("[Visp Tracker] : Initialisation Succeeded!")
        
        return "success"


class GetObjectPosition(ssm_state.ssmState):
    
    def __init__(self):
        ssm_state.ssmState.__init__(self, outcomes = ['success', 'fail'],
                                    io_keys = ['object_pose_file' , 'object_id', 'object_pose','target_point'])
        
        self.ObjPose_sub    = None
        self.RobotPose_sub  = None
        self.execution_      = False
        self.ObjectPose     = None
        self.RobotPose      = None
        
        
        
    
    def ObjectPose_cb(self, msg):
        if(self.execution_):
            if(not(msg.header.frame_id == "Object NOT FOUND")):
                self.ObjectPose = msg
            else:
                self.ObjectPose = None

    def RobotPose_cb(self, msg):
        if(self.execution_):
            self.RobotPose = msg

        
    def execution(self, ud):
        self.ObjPose_sub    = rospy.Subscriber("/vp_tracker/bracket_pose",PoseStamped, self.ObjectPose_cb)
        self.RobotPose_sub  = rospy.Subscriber("/lwr/pose_cart",PoseStamped, self.RobotPose_cb)
        rospy.sleep(2)
        self.ObjectPose = None
        self.RobotPose = None
        
        
        XMLFile   = ET.parse(get_pkg_dir_from_prefix(ud.object_pose_file))
        root = XMLFile.getroot()
        ToolCam        = ReadXmlPoint(root.find("ToolCam"))
        MoveOffset     = ReadXmlPoint(root.find("Obj"+ud.object_id))
        self.execution_ = True
        cpt = 0
        r = rospy.Rate(10)
        while ((self.ObjectPose == None) or (self.RobotPose == None)):
            if ((cpt > 100)): ##10seconds                
                rospy.logerr("[Visp Tracker] : Object Not Found !")
                self.execution_ = False
                return "fail"
            else:
                cpt = cpt + 1
                r.sleep()
                
        MBaseToTool0    = self.convertToMat(self.RobotPose)
        MTool0ToCamera  = self.convertToMat(ToolCam)
        MCameraToObject = self.convertToMat(self.ObjectPose)
        
        MOffsetToTake   = self.convertToMat(MoveOffset)
        
        Target = numpy.dot(MBaseToTool0,MTool0ToCamera)
        Target = numpy.dot(Target,MCameraToObject)
        ud.object_pose = self.convertFromMat(Target)
        Target = numpy.dot(Target,MOffsetToTake)
        
        PTarget = self.convertFromMat(Target)
        PTarget.header.stamp = rospy.Time.now()
        PTarget.pose.position.x = PTarget.pose.position.x/1000.0
        PTarget.pose.position.y = PTarget.pose.position.y/1000.0
        PTarget.pose.position.z = PTarget.pose.position.z/1000.0
        ud.target_point = PTarget
        
        rospy.loginfo("[Visp Tracker] : Target Acquired !")
        
        return "success"
    
    def convertToMat(self, Pose):
        px_ = Pose.pose.position.x
        py_ = Pose.pose.position.y
        pz_ = Pose.pose.position.z
        qx_ = Pose.pose.orientation.x
        qy_ = Pose.pose.orientation.y
        qz_ = Pose.pose.orientation.z
        qw_ = Pose.pose.orientation.w
            
        Mat = numpy.eye(4) ##Init
        Mat_ = QuatToMatrix(qx_, qy_, qz_, qw_)
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
    
    def convertFromMat(self, Mat):
        
        Pose = PoseStamped()
        quat = MatrixToQuat(Mat[0:3][0:3]) 
        
        Pose.pose.position.x = Mat[0][3]
        Pose.pose.position.y = Mat[1][3]
        Pose.pose.position.z = Mat[2][3]
        Pose.pose.orientation.x = quat[0]
        Pose.pose.orientation.y = quat[1]
        Pose.pose.orientation.z = quat[2]
        Pose.pose.orientation.w = quat[3]
        
        
        return Pose
'''    
class ComputeOffset(ssm_core.AirState):
    
    def __init__(self):
        ssm_core.AirState.__init__(self, io_keys = ['outcome' , 'data'])
        
        self.RobotPose_sub  = rospy.Subscriber("/lwr/pose_cart",PoseStamped, self.RobotPose_cb)
        self.execution      = False
        self.RobotPose      = None
        

    def RobotPose_cb(self, msg):
        if(self.execution):
            self.RobotPose = msg

        
    def execute(self, ud):
        self.RobotPose = None
        
        self.execution = True
        cpt = 0
        r = rospy.Rate(10)
        while (self.RobotPose == None):
            if ((cpt > 100)): ##10seconds                
                rospy.logerr("[LWR ARM] : Error while waiting for the cartesian position !")
                self.execution = False
                return self.ABORTED
            else:
                cpt = cpt + 1
                r.sleep()
                
        MBaseToTool0    = self.convertToMat(self.RobotPose)
        MObjectPosition = self.convertToMat(ud.data["ObjectPose"])
        Offset = numpy.dot(numpy.linalg.inv(MObjectPosition),MBaseToTool0)
        print(self.convertFromMat(Offset))
        
        return self.SUCCEEDED
    
    def convertToMat(self, Pose):
        px_ = Pose.pose.position.x
        py_ = Pose.pose.position.y
        pz_ = Pose.pose.position.z
        qx_ = Pose.pose.orientation.x
        qy_ = Pose.pose.orientation.y
        qz_ = Pose.pose.orientation.z
        qw_ = Pose.pose.orientation.w
            
        Mat = numpy.eye(4) ##Init
        Mat_ = QuatToMatrix(qx_, qy_, qz_, qw_)
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
    
    def convertFromMat(self, Mat):
        
        Pose = PoseStamped()
        quat = MatrixToQuat(Mat[0:3][0:3]) 
        
        Pose.pose.position.x = Mat[0][3]
        Pose.pose.position.y = Mat[1][3]
        Pose.pose.position.z = Mat[2][3]
        Pose.pose.orientation.x = quat[0]
        Pose.pose.orientation.y = quat[1]
        Pose.pose.orientation.z = quat[2]
        Pose.pose.orientation.w = quat[3]
        
        
        return Pose
'''






#End of file

