#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : controller.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import rospy
import time

from robotiq_s_model_control.msg._SModel_robot_output import SModel_robot_output
from robotiq_s_model_control.msg._SModel_robot_input import SModel_robot_input

from protocol import *

__all__ = ['RobotiqSModelController']

class RobotiqSModelController:
    
    def __init__(self):
        
        self._robotiq_s_model_pub = rospy.Publisher('SModelRobotOutput',
                                                    SModel_robot_output,
                                                    latch=True)
        
        self._robot_input = SModel_robot_input()
        
        rospy.Subscriber("SModelRobotInput", SModel_robot_input, self._robot_input_cb)
        
    def _robot_input_cb(self, msg):
        
        self._robot_input = msg
        
    def execute(self, robot_output, success_cond_cb, timeout=60):
        
        if not isinstance(robot_output, SModel_robot_output):
            rospy.logerr('Invalid data type: robot_output is not type C{SModel_robot_output} !')
            
        if not self.is_activated():
            init_res = self.activate(timeout)
            if init_res == ROBOTIQ_SUCCEEDED:
                #Robotiq is activated
                pass
            else:
                #Return error code 
                return init_res
        
        try:
            self._robotiq_s_model_pub.publish(robot_output)
        except Exception as e:
            cobble.logerr('RobotiqSModelController::publisher raised with exception "%s"'%e)
            return ROBOTIQ_ABORTED
        
        return self.wait_for_goal_reached(success_cond_cb, timeout)
    
    def wait_for_goal_reached(self, success_cond_cb, timeout):
        
        start_t = rospy.get_time()
        
        while not rospy.is_shutdown():
        #{
            status = success_cond_cb(self._robot_input)
            
            if (rospy.get_time() - start_t) > timeout:
            #{
                rospy.logerr('RobotiqSModelController::wait_for_goal_reached() timeout "%d" reached !'
                             %timeout)
                return ROBOTIQ_TIMEOUT
            #}End if
            elif status != ROBOTIQ_PENDING:
            #{
                if status == ROBOTIQ_SUCCEEDED:
                #{
                    return ROBOTIQ_SUCCEEDED
                #}
                elif status in ROBOTIQ_ERR_CODE:
                #{
                    return status
                #}End elif
                else:
                #{
                    rospy.sleep(0.1)
                #}End else
            #}
            else:
            #{
                rospy.sleep(0.1)
            #}
        #}End while
    
    def is_activated(self):
        
        if self._robot_input.gIMC == 3:
            return True
        else:
            return False
        
    def activate(self, timeout):
        
        try:
            self._robotiq_s_model_pub.publish(SModel_robot_output(rACT = 1,
                                                                  rGTO = 1,
                                                                  rSPA = 255,
                                                                  rFRA = 150)
                                              )
        except Exception as e:
            return ROBOTIQ_ABORTED
        
        start_t = rospy.get_time()
        
        while not rospy.is_shutdown():
        #{
            if (rospy.get_time() - start_t) > timeout:
            #{
                rospy.logerr('RobotiqSModelController::wait_for_initialization_done() timeout "%d secs" reached !'
                             %timeout)
                return ROBOTIQ_TIMEOUT
            #}End if
            elif self.is_activated():
                return ROBOTIQ_SUCCEEDED
            else:
                rospy.sleep(0.1)
        #}End while
        
        return ROBOTIQ_SUCCEEDED
        
def main():
    
    rospy.init_node('robotiq_s_model_controller_test')
    
    robotiq = RobotiqSModelController()
    robotiq.activate(timeout=10)
    
    def wide_openned(robot_input):
        
        if robot_input.gPRA == 0:
            if robot_input.gDTA == 3 and \
               robot_input.gDTB == 3 and \
               robot_input.gDTC == 3 :
                return ROBOTIQ_SUCCEEDED
        return ROBOTIQ_PENDING
    
    res = robotiq.execute(SModel_robot_output(rACT = 1,
                                              rGTO = 1,
                                              rSPA = 255,
                                              rFRA = 150,
                                              rPRA = 0,
                                              rMOD = 2),
                                              success_cond_cb=wide_openned)
    
    print res
    
if __name__ == '__main__':
    main()

#End of file
