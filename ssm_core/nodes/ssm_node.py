#!/usr/bin/env python

################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : Interpreter.py
# Authors : Ludovic DELVAL
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Ludovic DELVAL <ludovic.delval.external@airbus.com>
#
#
#
#
################################################################################

import rospy

from ssm_core.ssm_main import ssmMain
from std_msgs.msg import Empty
    
if __name__ == '__main__':
    
    rospy.init_node('SCXML', log_level=rospy.INFO)
    SSM = ssmMain()
    if(rospy.get_param('ssm_autostart', False) == True):
       if(SSM._init_SSM()):
          SSM.start(Empty)
    rospy.spin()
    
