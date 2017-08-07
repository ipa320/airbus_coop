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
from xml.etree import ElementTree as ET
from ssm_core import ssm_state
from geometry_msgs.msg  import PoseStamped
from controller import NavigationController


class MovePlatform(ssm_state.ssmState):
    
    def __init__(self):
        
        ssm_state.ssmState.__init__(self,io_keys = ["target_station","nav_timeout"],outcomes=["success"])
        
    def execution(self, ud):
        
        goal = ud.target_station
        timeout_ = int(ud.nav_timeout)
        if(timeout_ < 10):
            rospy.logwarn("[Navigation Skill] : Navigation Timeout set to 10 seconds !")
            timeout_ = 10
            
        controller = NavigationController(
                                       NavigationController.TYPE_SIMPLE_GOAL,
                                       self.preempt_requested)
        
        result = controller.move_pose_command(goal, rospy.Duration(timeout_))
        
        if result == NavigationController.SUCCEEDED:
            return "success"
        else:
            return "preempt"
    
class ScanMatcherPlatform(ssm_state.ssmState):
        
     def __init__(self):
        
        ssm_state.ssmState.__init__(self,io_keys = ["target_station","nav_timeout"],outcomes=["success"])
        
     def execution(self, ud):
        
        goal = ud.target_station
        timeout_ = int(ud.nav_timeout)
        if(timeout_ < 10):
            rospy.logwarn("[Navigation Skill] : Navigation Timeout set to 10 seconds !")
            timeout_ = 10
            
        controller = NavigationController(
                                       NavigationController.TYPE_LAZER_SACN_MATCHER,
                                       self.preempt_requested)
        
        result = controller.move_pose_command(goal, rospy.Duration(timeout_))
        
        if result == NavigationController.SUCCEEDED:
            return "success"
        else:
            return "preempt"

#End of file

