#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : micro_epsilon_controller.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

import rospy
from std_msgs.msg import Int16, Int8
from geometry_msgs.msg import Pose

import time
__all__=["MircoEpsilonController"]

# /laser_power max 100 50 0

class MircoEpsilonController:
    
    # Commands
    INITIALIZE    = 0
    CALC_PLAN_YZ  = 100
    CALC_PLAN_XY  = 101
    CALC_PLAN_XZ  = 102
    CALC_BASE_2P1 = 150
    CALC_BASE_3P  = 151
    CALC_INTERSECTION = 103
    
    #Old interface
    COMPUTE_BASE_101 = 101
    
    LASER_POWER_OFF  = 0
    LASER_POWER_HALF = 50
    LASER_POWER_ON   = 100
    
    # Status
    BASE_THEO_RECEIVED = 0
    INITIALIZED        = 1
    SUCCEEDED          = 3
    
    #Errors
    ERROR       = -1
    TIMEOUT     = -2
    PREEMPTED   = -9
    REQUEST_UNKNOWN      = -1
    POINTS_CLOUD_TO_OLD  = -10
    TF_ERROR             = -11
    R_PLAN_To_BIG        = -101
    PLAN_NOT_LEARN       = -102
    DATA_UNKNOWN_TO_CALC = -150
    
    ERR_CODE = [ERROR,
                TIMEOUT,
                PREEMPTED,
                REQUEST_UNKNOWN,
                POINTS_CLOUD_TO_OLD,
                TF_ERROR,
                R_PLAN_To_BIG,
                PLAN_NOT_LEARN,
                DATA_UNKNOWN_TO_CALC]
    
    def __init__(self, preempt_cb):
        
        self._is_preempted_cb = preempt_cb
        
        self.power_state     = self.LASER_POWER_OFF
        self.laser_power_pub = rospy.Publisher("/laser_power",
                                               Int8,
                                               queue_size=1)
        
        self.micro_epsilon_pub = rospy.Publisher("/trigger_req",
                                                  Int16,
                                                  queue_size=1)
        
        self.answer = 0
        self.micro_epsilon_sub = rospy.Subscriber("/trigger_ans",
                                                   Int16,
                                                   self.micro_epsilon_states_cb)
        
        self._base_theo_pub = rospy.Publisher("/base_theo",
                                              Pose,
                                              queue_size=1)
        
        self._wait_for_available_connections([self.laser_power_pub,
                                              self.micro_epsilon_pub,
                                              self.micro_epsilon_sub],
                                              rospy.Duration(0.1))
        
        self.laser_power_command(self.LASER_POWER_OFF)
        
    def _wait_for_available_connections(self,
                                        connections = [],
                                        timeout=rospy.Duration(5)):
        
        r = rospy.Rate(10)
        initial_time = rospy.get_rostime()
        
        while not rospy.is_shutdown():
            if (rospy.get_rostime() - initial_time) > timeout:
                rospy.logerr(self.__class__.__name__+ \
                             " checks topics connection -> Timeout reached!")
                return False
            available = 1
            for conn in connections:
                available *= (int)(conn.impl.has_connections())
            if available:
                rospy.loginfo(self.__class__.__name__+ \
                             " all topics are connected.")
                return True
            else:
                r.sleep()
        return False
    
    def micro_epsilon_states_cb(self, ans):
        self.answer = ans.data
        
    def laser_power_command(self, cmd):
        
        try:
            self.laser_power_pub.publish(Int8(cmd))
        except Exception as ex:
            rospy.logerr(self.__class__.__name__+
                         "::laser_power_command raised with exception :%s"%str(ex))
            self.power_state = self.LASER_POWER_OFF
            return self.ERROR
        
        if cmd == self.LASER_POWER_ON:
            time.sleep(2)
        else:
            time.sleep(0.5)
        
        self.power_state = cmd
        
        return self.SUCCEEDED
    
    def communicate_base_theorical(self,
                                   base,
                                   timeout = rospy.Duration(30)):
        
        if not isinstance(base, Pose):
            rospy.logerr("Bad data type from communicate_base_theorical")
            return self.ERROR
        
        try:
            self._base_theo_pub.publish(base)
        except Exception as e:
            rospy.logerr("MircoEpsilonController publisher raised with exception %s"%str(e))
            return self.ERROR
        
        return self.wait_for(self.BASE_THEO_RECEIVED, timeout)
        
    def command(self,
                cmd,
                condition = None,
                timeout = rospy.Duration(30)):
        
        try:
            self.micro_epsilon_pub.publish(Int16(cmd))
        except Exception as e:
            rospy.logerr("MircoEpsilonController publisher raised with exception %s"%str(e))
            return self.ERROR
        
        if condition is not None:
            return self.wait_for(condition, timeout)
        
        return self.SUCCEEDED
    
    def wait_for(self, condition, timeout = rospy.Duration(30)):
    
        r = rospy.Rate(100)
        initial_time = rospy.get_rostime()
        
        while not rospy.is_shutdown():
            
            if self._is_preempted_cb():
                return self.PREEMPTED
            elif (rospy.get_rostime() - initial_time) > timeout:
                return self.TIMEOUT
            elif self.answer == condition:
                return self.SUCCEEDED
            elif self.answer < 0:
                rospy.logerr("MircoEpsilonController returned error code %i"%self.answer)
                return self.answer
            else:
                r.sleep()
                
        return self.SUCCEEDED
    
if __name__ == '__main__':
    
    rospy.init_node('utt_mirco_epsilon_node', log_level=rospy.INFO)
    
    def p():
        return False
    
    c = MircoEpsilonController(p)
    
    c.command(MircoEpsilonController.INITIALIZE, MircoEpsilonController.INITIALIZED)
    
    
    
