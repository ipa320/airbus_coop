#!/usr/bin/env python

################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : time_skill.py
# Authors : Ludovic DELVAL
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Ludovic DELVAL <ludovic.delval.external@airbus.com>
#
#
################################################################################

import rospy
from ssm_core import ssm_state


class waitTime(ssm_state.ssmState):

    def __init__(self):
        ssm_state.ssmState.__init__(self, io_keys=['waitsec'], outcomes=['success'])
        
    def execution(self, ud):
        temps=int(ud.waitsec)
        r = rospy.Rate(100)
        cpt=0
        while (cpt < temps*100 or self.preempt_requested()):
            cpt = cpt + 1
            r.sleep()
            
        if(cpt==(temps*100)):
            return 'success'
        else:
            return 'preempt'
 
