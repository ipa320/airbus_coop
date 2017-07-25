#!/usr/bin/env python

import datetime
import rospy

def StrTimeStamped():
    return datetime.datetime.now().strftime("%Y_%m_%d_%H:%M:%S.%f   - ")

class onEntry():
    
    def __init__(self, logs={}, script=None):
        self._logs = logs
        self._script = script
    
    def execute(self, ud):
        file = open(ud.logfile, "a")
        for log in self._logs:
            if(log == "outcome"):
                rospy.logwarn("[SSM : onEntry] : Log of the outcome expected. This is not possible in <onentry>")
            elif(log == ""):
                file.write(StrTimeStamped() + self._logs[log] +"\n")
            else:
                file.write(StrTimeStamped() + self._logs[log] + " " +str(ud[log])+"\n")
        file.close()
        
        if(self._script is not None):        
            try:
                exec(self._script)
            except RuntimeError as msg:
                rospy.logerr("[SSM : onEntry] : %s"%msg)
            
class onExit():
    
    def __init__(self, logs={}, script=None):
        self._logs = logs
        self._script = script
    
    def execute(self, ud, outcome_):
        outcome = outcome_          
        if(self._script is not None):        
            try:
               exec(self._script)
            except RuntimeError as msg:
                rospy.logerr("[SSM : onExit] : %s"%msg)
                
        file = open(ud.logfile, "a")
        for log in self._logs:
            if(log == "outcome"):
                file.write(StrTimeStamped() + self._logs[log] + " " +str(outcome)+"\n")
            elif(log == ""):
                file.write(StrTimeStamped() + self._logs[log] +"\n")
            else:
                file.write(StrTimeStamped() + self._logs[log] + " " +str(ud[log])+"\n")
        file.close()
        return outcome
