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
import ssm_core

from controller import MircoEpsilonController
import ros

__all__ = ['MircoEpsilonState',
           'MircoEpsilonServiceState',
           'MircoEpsilonLaserOn',
           'MircoEpsilonLaserOff',
           'MircoEpsilonInitialization',
           'MircoEpsilonSingleShot',
           'MircoEpsilonCalcPlanYZ',
           'MircoEpsilonCalcPlanXY',
           'MircoEpsilonCalcIntersection',
           'MircoEpsilonCalcBase']

class MircoEpsilonState(air_smach.AirState):
    
    def __init__(self,
                 outcomes = [],
                 input_keys = [],
                 output_keys = [],
                 io_keys = [],
                 timeout=None,
                 widget_key=None):
        
        air_smach.AirState.__init__(
                 self,
                 outcomes    = outcomes,
                 input_keys  = input_keys,
                 output_keys = output_keys,
                 io_keys     = io_keys,
                 timeout     = timeout,
                 widget_key  = widget_key)
        
        self.controller = MircoEpsilonController(self.preempt_requested)
        
    def get_controller(self):
        return self.controller

class MircoEpsilonServiceState(air_smach.AirServiceState):
    
    def __init__(self,
                 # Service
                 service_name,
                 service_class,
                 service_timeout,
                 # Keys
                 outcomes = [],
                 input_keys = [],
                 output_keys = [],
                 io_keys = [],
                 timeout=None,
                 widget_key=None):
        
        air_smach.AirServiceState.__init__(
                 self,
                 service_name    = service_name,
                 service_class   = service_class,
                 service_timeout = service_timeout,
                 outcomes        = outcomes,
                 input_keys      = input_keys,
                 output_keys     = output_keys,
                 io_keys         = io_keys,
                 timeout         = timeout,
                 widget_key      = widget_key)
        
        self.controller = MircoEpsilonController(self.preempt_requested)
        
    def get_controller(self):
        return self.controller
    
class MircoEpsilonLaserOn(MircoEpsilonState):
    
    def __init__(self):
        
        MircoEpsilonState.__init__(self, timeout=rospy.Duration(60))
        
        self._micro_epsilon = self.get_controller()
        
    def execute(self, ud):
        
        res = self._micro_epsilon.laser_power_command(self._micro_epsilon.LASER_POWER_ON)
        
        if res == self._micro_epsilon.SUCCEEDED:
            return self.SUCCEEDED
        else:
            self.logerr(self.__class__.__name__+" cannot set laser to power on !")
            return self.ABORTED
        
class MircoEpsilonLaserOff(MircoEpsilonState):
    
    def __init__(self):
        
        MircoEpsilonState.__init__(self, timeout=rospy.Duration(60))
        
        self._micro_epsilon = self.get_controller()
        
    def execute(self, ud):
        
        res = self._micro_epsilon.laser_power_command(self._micro_epsilon.LASER_POWER_OFF)
        
        if res == self._micro_epsilon.SUCCEEDED:
            return self.SUCCEEDED
        else:
            self.logerr(self.__class__.__name__+" cannot set laser to power off !")
            return self.ABORTED
    
class MircoEpsilonInitialization(MircoEpsilonState):
    
    def __init__(self):
        
        MircoEpsilonState.__init__(self,
                                    output_keys=['shot_id'],
                                    timeout=rospy.Duration(60))
        
        self._micro_epsilon = self.get_controller()
        
    def execute(self, ud):
        
        ud.shot_id = 0
        
        res = self._micro_epsilon.command(MircoEpsilonController.INITIALIZE,
                                          MircoEpsilonController.INITIALIZED,
                                          rospy.Duration(10))
        
        if res == self._micro_epsilon.SUCCEEDED:
            return self.SUCCEEDED
        else:
            self.logerr(self.__class__.__name__+" returned error code %i !"%res)
            return self.ABORTED
        

class MircoEpsilonSingleShot(MircoEpsilonState):
    
    def __init__(self):
        
        MircoEpsilonState.__init__(self,
                                    io_keys=['shot_id'],
                                    timeout=rospy.Duration(60))
        
        self._micro_epsilon = self.get_controller()
        
    def execute(self, ud):
        
        ud.shot_id += 1
        
        if self.power_on() != self.SUCCEEDED:
            return self.ABORTED
        
        print 'MircoEpsilonSingleShot cmd :',ud.shot_id
        res = self._micro_epsilon.command(ud.shot_id,
                                          ud.shot_id,
                                          rospy.Duration(30))
        
        if res == self._micro_epsilon.SUCCEEDED:
            return self.power_off()
        else:
            self.power_off()
            self.logerr(self.__class__.__name__+" returned error code %i !"%res)
            return self.ABORTED
        
    def power_on(self):
        
        res = self._micro_epsilon.laser_power_command(self._micro_epsilon.LASER_POWER_ON)
        
        if res == self._micro_epsilon.SUCCEEDED:
            return self.SUCCEEDED
        else:
            self.logerr(self.__class__.__name__+" cannot set laser to power on !")
            return self.ABORTED
        
    def power_off(self):
        
        res = self._micro_epsilon.laser_power_command(self._micro_epsilon.LASER_POWER_OFF)
        
        if res == self._micro_epsilon.SUCCEEDED:
            return self.SUCCEEDED
        else:
            self.logerr(self.__class__.__name__+" cannot set laser to power off !")
            return self.ABORTED
        
class MircoEpsilonCalcPlanYZ(MircoEpsilonState):
    
    def __init__(self):
        
        MircoEpsilonState.__init__(self,
                                   timeout=rospy.Duration(60))
        
        self._micro_epsilon = self.get_controller()
        
    def execute(self, ud):
        
        res = self._micro_epsilon.command(MircoEpsilonController.CALC_PLAN_YZ,
                                          MircoEpsilonController.CALC_PLAN_YZ,
                                          rospy.Duration(10))
        
        if res == self._micro_epsilon.SUCCEEDED:
            return self.SUCCEEDED
        else:
            self.logerr(self.__class__.__name__+" returned error code %i !"%res)
            return self.ABORTED
        
class MircoEpsilonCalcPlanXY(MircoEpsilonState):
    
    def __init__(self):
        
        MircoEpsilonState.__init__(self,
                                   timeout=rospy.Duration(60))
        
        self._micro_epsilon = self.get_controller()
        
    def execute(self, ud):
        
        res = self._micro_epsilon.command(MircoEpsilonController.CALC_PLAN_XY,
                                          MircoEpsilonController.CALC_PLAN_XY,
                                          rospy.Duration(10))
        
        if res == self._micro_epsilon.SUCCEEDED:
            return self.SUCCEEDED
        else:
            self.logerr(self.__class__.__name__+" returned error code %i !"%res)
            return self.ABORTED
        
class MircoEpsilonCalcIntersection(MircoEpsilonState):
    
    def __init__(self):
        
        MircoEpsilonState.__init__(self,
                                   timeout=rospy.Duration(60))
        
        self._micro_epsilon = self.get_controller()
        
    def execute(self, ud):
        
        res = self._micro_epsilon.command(MircoEpsilonController.CALC_INTERSECTION,
                                          MircoEpsilonController.CALC_INTERSECTION,
                                          rospy.Duration(10))
        
        if res == self._micro_epsilon.SUCCEEDED:
            return self.SUCCEEDED
        else:
            self.logerr(self.__class__.__name__+" returned error code %i !"%res)
            return self.ABORTED
        
class MircoEpsilonCalcBase(MircoEpsilonState):
    
    def __init__(self):
        
        MircoEpsilonState.__init__(self,
                                   timeout=rospy.Duration(60))
        
        self._micro_epsilon = self.get_controller()
        
    def execute(self, ud):
        
        res = self._micro_epsilon.command(MircoEpsilonController.CALC_BASE_2P1,
                                          MircoEpsilonController.CALC_BASE_2P1,
                                          rospy.Duration(10))
        
        if res == self._micro_epsilon.SUCCEEDED:
            return self.SUCCEEDED
        else:
            self.logerr(self.__class__.__name__+" returned error code %i !"%res)
            return self.ABORTED
        
class MircoEpsilonComputeBaseOld(MircoEpsilonState):
    
    def __init__(self):
        
        MircoEpsilonState.__init__(self,
                                   output_keys=['base_probed'],
                                   timeout=rospy.Duration(60))
        
        self._micro_epsilon = self.get_controller()
        
    def execute(self, ud):
        
        res = self._micro_epsilon.command(MircoEpsilonController.COMPUTE_BASE_101,
                                          MircoEpsilonController.COMPUTE_BASE_101,
                                          rospy.Duration(10))
        
        base = self._micro_epsilon.get_base_probed()
        
        ud.base_probed = [base.position.x,
                          base.position.y,
                          base.position.z,
                          base.orientation.x,
                          base.orientation.y,
                          base.orientation.z,
                          0.]
        print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
        print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
        print base
        print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
        print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
        
        if res == self._micro_epsilon.SUCCEEDED:
            return self.SUCCEEDED
        else:
            self.logerr(self.__class__.__name__+" returned error code %i !"%res)
            return self.ABORTED
    

#End of file

