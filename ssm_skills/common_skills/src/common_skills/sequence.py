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

import rospy
import xml.etree.ElementTree as etree
from pyqt_agi_extend.QtAgiCore import get_pkg_dir_from_prefix
from common_lib.pt_xml import read_cartesian, read_nav_point


class readTargetPoint(ssm_state.ssmState):

    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["success"], 
                                    io_keys=["target_point","sequence_file","tool","station_id", "point_id"])
        
    def execution(self, ud):
        file_ = open(get_pkg_dir_from_prefix(ud.sequence_file)).read()
        root = etree.fromstring(file_)
        pt = root.find("./nav_points/nav_point[@id='"+ud.station_id+"']/points/point[@id='"+ud.point_id+"']")
        if(pt == None):
            rospy.logerr("[Read Pt XML] : Point +"+ud.point_id+" not found !")
            return "preempt"
        else:
            ud.target_point, ud.tool, ud.point_id, process = read_cartesian(pt)
            rospy.loginfo("[Arm Sequence XML] : Going to point : "+str(ud.point_id))
            return "success"
 


class readArmSequence(ssm_state.ssmState):

    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["next0" , "next1" , "out"], 
                                    io_keys=["target_point","sequence_file","tool","target_station","station_id", "point_id"])
        
        self.sequence = None
        self.reset = False
        
    def execution(self, ud):
        if(self.reset == False):
            self.reset = True
            file_ = open(get_pkg_dir_from_prefix(ud.sequence_file)).read()
            self.root = etree.fromstring(file_)
            self.sequence = self.root.findall("./nav_points/nav_point[@id='"+ud.station_id+"']/points/point")
            print(self.sequence)
        if(len(self.sequence) == 0):
            rospy.loginfo("[Arm Sequence XML] : Sequence finished !")
            self.reset = False
            return "out"
        else:
            ud.target_point, ud.tool, ud.point_id, process = read_cartesian(self.sequence[0])
            outcome = "next_process" + process   
            rospy.loginfo("[Arm Sequence XML] : Going to point : "+str(ud.point_id))
            del self.sequence[0]
            return outcome
        
class readArmSequence3Process(ssm_state.ssmState):

    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["next_process0" , "next_process1", "next_process2" , "out"], 
                                    io_keys=["target_point","sequence_file","tool","target_station","station_id", "point_id"])
        
        self.sequence = None
        self.reset = False
        
    def execution(self, ud):
        if(self.reset == False):
            self.reset = True
            file_ = open(get_pkg_dir_from_prefix(ud.sequence_file)).read()
            self.root = etree.fromstring(file_)
            self.sequence = self.root.findall("./nav_points/nav_point[@id='"+ud.station_id+"']/points/point")
            print(self.sequence)
        if(len(self.sequence) == 0):
            rospy.loginfo("[Arm Sequence XML] : Sequence finished !")
            self.reset = False
            return "out"
        else:
            ud.target_point, ud.tool, ud.point_id, process = read_cartesian(self.sequence[0])
            outcome = "next_process" + str(process) 
            rospy.loginfo("[Arm Sequence XML] : Going to point : "+str(ud.point_id))
            del self.sequence[0]
            return outcome
        
class readArmSequence4Process(ssm_state.ssmState):

    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["next_process0" , "next_process1", "next_process2", "next_process3" , "out"], 
                                    io_keys=["target_point","sequence_file","tool","target_station","station_id", "point_id"])
        
        self.sequence = None
        self.reset = False
        
    def execution(self, ud):
        if(self.reset == False):
            self.reset = True
            file_ = open(get_pkg_dir_from_prefix(ud.sequence_file)).read()
            self.root = etree.fromstring(file_)
            self.sequence = self.root.findall("./nav_points/nav_point[@id='"+ud.station_id+"']/points/point")
            print(self.sequence)
        if(len(self.sequence) == 0):
            rospy.loginfo("[Arm Sequence XML] : Sequence finished !")
            self.reset = False
            return "out"
        else:
            ud.target_point, ud.tool, ud.point_id, process = read_cartesian(self.sequence[0])
            outcome = "next_process" + str(process) 
            rospy.loginfo("[Arm Sequence XML] : Going to point : "+str(ud.point_id))
            del self.sequence[0]
            return outcome



        
class readNavigationSequence(ssm_state.ssmState):
    ##Describe a loop which check if i_for < cond_for
    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["next","out"], 
                                    io_keys=["sequence_file","target_station", "station_id"])
        
        self.sequence = None
        self.reset = False
        
    def execution(self, ud):
        if(self.reset == False):
            self.reset = True
            file_ = open(get_pkg_dir_from_prefix(ud.sequence_file)).read()
            self.root = etree.fromstring(file_)
            self.sequence = self.root.findall("./nav_points/nav_point")
        if(len(self.sequence) == 0):
            rospy.loginfo("[Navigation Sequence XML] : Sequence finished !")
            self.reset = True
            return "out"
        else:
            ud.target_station, ud.station_id = read_nav_point(self.sequence[0])
            rospy.loginfo("[Navigation Sequence XML] : Going to station : "+str(ud.station_id))
            del self.sequence[0]
            return "next"
