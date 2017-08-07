#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : unittest.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import rospy
import air_smach
import time

from controller import MircoEpsilonController
from states import MircoEpsilonLaserOn, MircoEpsilonLaserOff

class SMUnittest(air_smach.AirStateMachine):
    
    def __init__(self):
        
        air_smach.AirStateMachine.__init__(self)
        
        with self:
            
            self.add("MircoEpsilonLaserOn",MircoEpsilonLaserOn(),
                     {MircoEpsilonLaserOn.SUCCEEDED : "MircoEpsilonLaserOff",
                      MircoEpsilonLaserOn.ABORTED   : self.ABORTED})
             
            self.add("MircoEpsilonLaserOff",MircoEpsilonLaserOff(),
                     {MircoEpsilonLaserOff.SUCCEEDED : "MircoEpsilonLaserOn",
                      MircoEpsilonLaserOff.ABORTED   : self.ABORTED})
            
    
if __name__ == '__main__':
    
    rospy.init_node('utt_mirco_epsilon_node', log_level=rospy.INFO)
    
    air_smach.init_server(server_name='/mirco_epsilon_skill', verbose=True)
     
    sm = SMUnittest()
     
    sis = air_smach.IntrospectionServer(air_smach.get_server_name(), sm, "ROOT")
    sis.start()
    sm.exec_()
    sis.stop()
    
