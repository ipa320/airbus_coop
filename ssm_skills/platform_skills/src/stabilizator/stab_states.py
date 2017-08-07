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
from ssm_core import ssm_state
import time


from stabilization import StabilisationController, \
                          StabilizatorsCtrl, \
                          StabilizatorsResult

class StabilisationCommonSkill(ssm_state.ssmState):
    def __init__(self, command):
        
        ssm_state.ssmState.__init__(self,
                                    io_keys = [],
                                    outcomes = ['success'])
        
        self._command = command
        
    def execution(self, ud):
        
        controller = StabilisationController(rospy.Duration(10))
        
        ctrl = StabilizatorsCtrl()
        ctrl.rear_left   = self._command
        ctrl.rear_right  = self._command
        ctrl.front_left  = self._command
        ctrl.front_right = self._command
        
        result = controller.command(ctrl)
        
        if self.preempt_requested():
            return 'preempt'
        elif result == StabilizatorsResult.SUCCEEDED:
            return 'success'
        else:
            return 'preempt'

class InitStabilizatorSkill(StabilisationCommonSkill):
    def __init__(self):
        StabilisationCommonSkill.__init__(self, StabilisationController.INITIALIZE)

class StabilizeSkill(StabilisationCommonSkill):
    def __init__(self):
        StabilisationCommonSkill.__init__(self, StabilisationController.STABILIZE)
        
class UnStabilizeSkill(StabilisationCommonSkill):
    def __init__(self):
        StabilisationCommonSkill.__init__(self, StabilisationController.UNSTABILIZE)
        
#End of file

