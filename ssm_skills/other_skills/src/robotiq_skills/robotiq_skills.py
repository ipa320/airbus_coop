#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : s_model_skills.py
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

from protocol import *
from controller import RobotiqSModelController
from robotiq_s_model_control.msg._SModel_robot_output import SModel_robot_output

__all__ = ['Activation',
           'TemplateSkill',
           'Openning',
           'Closing',
           'Grasping',
           'WideOpenning',
           'PositioningFingers']

class Activation(ssm_state.ssmState):
    
    def __init__(self):
        
        ssm_state.ssmState.__init__(self, io_keys = ['robotiq_command'], outcomes = ["success", "fail"])
        
        self._robot_controller = RobotiqSModelController()
        
    def execution(self, ud):
        
        ud.robotiq_command = SModel_robot_output(rACT = 1,
                                         rGTO = 1,
                                         rSPA = 255,
                                         rFRA = 50)
        
        if self._robot_controller.activate(60) == ROBOTIQ_SUCCEEDED:
            return "success"
        else:
            return "fail"


class TemplateSkill(ssm_state.ssmState):
    
    def __init__(self,
                 commands_kws,
                 success_cond_cb,
                 timeout_secs=60):
        """! You must be activated robotiq before using this skill."""
        
        ssm_state.ssmState.__init__(self, io_keys = ['robotiq_command'], outcomes = ["success", "fail"])
        
        self._commands         = commands_kws
        self._cb               = success_cond_cb
        self._timeout          = timeout_secs
        self._robot_controller = RobotiqSModelController()
        
    def execution(self, ud):
        
        #rospy.sleep(3.5)
        
        for key, value in self._commands.items():
            setattr(ud.robotiq_command, key, value)
        
        result = self._robot_controller.execute(ud.robotiq_command,
                                                self._cb,
                                                self._timeout)
        
        if result == ROBOTIQ_SUCCEEDED:
            return "success"
        elif result in ROBOTIQ_ERR_CODE:
            return "fail"
        else:
            return "preempt"

        
class Openning(TemplateSkill):
    
    def __init__(self):
        
        def openned_cond_cb(robot_input):
            
            if robot_input.gPRA == CMD_OPEN:
                if robot_input.gDTA == FINGER_OPENNED and \
                   robot_input.gDTB == FINGER_OPENNED and \
                   robot_input.gDTC == FINGER_OPENNED :
                    return ROBOTIQ_SUCCEEDED
                elif robot_input.gDTA == FINGER_GRASPED or \
                     robot_input.gDTB == FINGER_GRASPED or \
                     robot_input.gDTC == FINGER_GRASPED :
                    return ROBOTIQ_ABORTED
                else:
                    return ROBOTIQ_RUNNING
            else:
                return ROBOTIQ_RUNNING
        
        TemplateSkill.__init__(self,
                               {'rPRA':CMD_OPEN},
                               openned_cond_cb,
                               60)
        
class Closing(TemplateSkill):
    
    def __init__(self):
        
        def closed_cond_cb(robot_input):
            
            if robot_input.gPRA == CMD_CLOSE:
                if robot_input.gDTA == FINGER_CLOSED and \
                   robot_input.gDTB == FINGER_CLOSED and \
                   robot_input.gDTC == FINGER_CLOSED :
                    return ROBOTIQ_SUCCEEDED
                elif robot_input.gDTA == FINGER_GRASPED or \
                     robot_input.gDTB == FINGER_GRASPED or \
                     robot_input.gDTC == FINGER_GRASPED :
                    return ROBOTIQ_ABORTED
                else:
                    return ROBOTIQ_RUNNING
            else:
                return ROBOTIQ_RUNNING
        
        TemplateSkill.__init__(self,
                               {'rPRA':CMD_CLOSE},
                               closed_cond_cb,
                               60)
        
class Grasping(TemplateSkill):
    
    def __init__(self):
        
        def grasped_cond_cb(robot_input):
            
            if robot_input.gPRA == CMD_CLOSE:
                if robot_input.gDTA == FINGER_GRASPED and \
                   robot_input.gDTB == FINGER_GRASPED and \
                   robot_input.gDTC == FINGER_GRASPED :
                    return ROBOTIQ_SUCCEEDED
                elif robot_input.gDTA == FINGER_CLOSED or \
                     robot_input.gDTB == FINGER_CLOSED or \
                     robot_input.gDTC == FINGER_CLOSED :
                    return ROBOTIQ_ABORTED
                else:
                    return ROBOTIQ_RUNNING
            else:
                return ROBOTIQ_RUNNING
        
        TemplateSkill.__init__(self,
                               {'rPRA':CMD_CLOSE},
                               grasped_cond_cb,
                               60)

class PinchMode(TemplateSkill):
    
    def __init__(self):
        
        def openned_cond_cb(robot_input):
            
            if robot_input.gPRA == CMD_OPEN:
                if robot_input.gDTA == FINGER_OPENNED and \
                   robot_input.gDTB == FINGER_OPENNED and \
                   robot_input.gDTC == FINGER_OPENNED :
                    return ROBOTIQ_SUCCEEDED
                elif robot_input.gDTA == FINGER_GRASPED or \
                     robot_input.gDTB == FINGER_GRASPED or \
                     robot_input.gDTC == FINGER_GRASPED :
                    return ROBOTIQ_ABORTED
                else:
                    return ROBOTIQ_RUNNING
            else:
                return ROBOTIQ_RUNNING

        
        TemplateSkill.__init__(self,
                               {'rPRA':CMD_OPEN,'rMOD':1},
                               openned_cond_cb,
                               60)

class BasicMode(TemplateSkill):
        
    def __init__(self):
        
        def openned_cond_cb(robot_input):
            
            if robot_input.gPRA == CMD_OPEN:
                if robot_input.gDTA == FINGER_OPENNED and \
                   robot_input.gDTB == FINGER_OPENNED and \
                   robot_input.gDTC == FINGER_OPENNED :
                    return ROBOTIQ_SUCCEEDED
                elif robot_input.gDTA == FINGER_GRASPED or \
                     robot_input.gDTB == FINGER_GRASPED or \
                     robot_input.gDTC == FINGER_GRASPED :
                    return ROBOTIQ_ABORTED
                else:
                    return ROBOTIQ_RUNNING
            else:
                return ROBOTIQ_RUNNING

        
        TemplateSkill.__init__(self,
                               {'rPRA':CMD_OPEN,'rMOD':0},
                               openned_cond_cb,
                               60)


        
class WideOpenning(TemplateSkill):
    
    def __init__(self):
        
        def wide_openned_cond_cb(robot_input):
            
            if robot_input.gPRA == 0:
                if robot_input.gDTA == FINGER_OPENNED and \
                   robot_input.gDTB == FINGER_OPENNED and \
                   robot_input.gDTC == FINGER_OPENNED :
                    return ROBOTIQ_SUCCEEDED
            else:
                return ROBOTIQ_RUNNING
            
        TemplateSkill.__init__(self,
                               {'rPRA':0,'rMOD':2},
                               wide_openned_cond_cb,
                               60)
        
class PositioningFingers(TemplateSkill):
    
    def __init__(self, position = 100):
        
        def position_reached_cond_cb(robot_input):
            
            if robot_input.gPRA == position:
                if robot_input.gDTA == FINGER_OPENNED and \
                   robot_input.gDTB == FINGER_OPENNED and \
                   robot_input.gDTC == FINGER_OPENNED :
                    return ROBOTIQ_SUCCEEDED
            else:
                return ROBOTIQ_RUNNING
        
        TemplateSkill.__init__(self,
                               {'rPRA':position},
                               position_reached_cond_cb,
                               60)

#End of file

