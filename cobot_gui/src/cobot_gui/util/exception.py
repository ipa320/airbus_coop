#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : exception.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import rospy
import inspect


def stdmsg(frame_desc=[], msg="Unknow"):
    
    if str(frame_desc[1]) == "<module>":
        frame_desc[1] = "__main__"
    
    msg = '[MSG]:%s [FILE]:%s [IN]:%s() [LINE]:%s'%(msg,
                                                   str(frame_desc[0]),
                                                   str(frame_desc[1]),
                                                   str(frame_desc[2]))
    return msg

## @package: exception
##
## @version 1.0
## @author  Matignon Martin
## @date    Last modified 30/04/2014

## @class CobotGuiException
## @brief Object for create an exception.
class CobotGuiException(Exception):
    
    def __init__(self, msg):
        self.msg = msg
        
        callerframerecord = inspect.stack()[1]
        frame = callerframerecord[0]
        info = inspect.getframeinfo(frame)
        filepyname = info.filename.split('/')
        
        rospy.logerr(stdmsg([filepyname[-1],info.function,info.lineno],msg))
        
    def __str__(self):
        return repr(self.msg)
    
if __name__ == "__main__":
    
    rospy.init_node('utt_cobot_gui_exception')
    
    try:
        x = 5/0
    except Exception as e:
        raise CobotGuiException(e)
    