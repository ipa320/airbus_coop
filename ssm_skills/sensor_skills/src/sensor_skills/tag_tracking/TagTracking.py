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
import ssm_core
import time
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D, PoseStamped
from xml.etree import ElementTree as ET
from tf import TransformListener
from read_xml import *
from convertion_tools import *
from pyqt_agi_extend.QtAgiCore import get_pkg_dir_from_prefix

class GetTagPosition(ssm_core.AirState):
    
    def __init__(self):
        ssm_core.AirState.__init__(self, io_keys = ['outcome' , 'data'])
        
        self.TagReq_pub     = rospy.Publisher("/tagseeker/request",Int16, queue_size=1)
        self.TagPose_sub    = rospy.Subscriber("/tagseeker/position_object",PoseStamped, self.TagPose_cb)
        self.RobotPose_sub  = rospy.Subscriber("/pose_cart",PoseStamped, self.RobotPose_cb)
        self.execution      = False
        self.TagPose        = None
        self.RobotPose      = None
        
        
        
    
    def TagPose_cb(self, msg):
        if(self.execution):
            if(not(msg.header.frame_id == "TAG NOT FOUND")):
                self.TagPose = msg
            else:
                self.TagPose = None

    def RobotPose_cb(self, msg):
        if(self.execution):
            self.RobotPose = msg

        
    def execute(self, ud):
        self.TagPose = None
        self.RobotPose = None
        
        
        XMLFile   = ET.parse(get_pkg_dir_from_prefix(ud.data["ObjectPose_file"]))
        root = XMLFile.getroot()
        ToolCam        = ReadXmlPoint(root.find("ToolCam"))
        ObjectPose     = ReadXmlPoint(root.find("Obj"+ud.data["ObjTarget"]))
        self.execution = True
        cpt = 0
        r = rospy.Rate(10)
        self.TagReq_pub.publish(int(ud.data["ObjTarget"]))
        while ((self.TagPose == None) or (self.RobotPose == None)):
            if ((cpt > 100)): ##10seconds                
                rospy.logerr("[TagTracker] : Tag Not Found !")
                self.execution = False
                return self.ABORTED
            else:
                cpt = cpt + 1
                r.sleep()
                
        MBaseToTool0    = self.convertToMat(self.RobotPose)
        MTool0ToCamera  = self.convertToMat(ToolCam)
        MCameraToObject = self.convertToMat(self.TagPose)
        
        MOffsetToTake   = self.convertToMat(ObjectPose)
        
        Target = numpy.dot(MBaseToTool0,MTool0ToCamera)
        Target = numpy.dot(Target,MCameraToObject)
        ud.data["ObjectPose"] = self.convertFromMat(Target)
        Target = numpy.dot(Target,MOffsetToTake)
        
        PTarget = self.convertFromMat(Target)
        PTarget.header.stamp = rospy.Time.now()
        ud.data["TargetPose"] = PTarget
        ud.outcome = "succeeded"
        rospy.loginfo("[TagTracker] : Target Acquired !")
        
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
    
class ComputeOffset(ssm_core.AirState):
    
    def __init__(self):
        ssm_core.AirState.__init__(self, io_keys = ['outcome' , 'data'])
        
        self.RobotPose_sub  = rospy.Subscriber("/pose_cart",PoseStamped, self.RobotPose_cb)
        self.execution      = False
        self.RobotPose      = None
        

    def RobotPose_cb(self, msg):
        if(self.execution):
            self.RobotPose = msg

        
    def execute(self, ud):
        self.TagPose = None
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







#End of file

