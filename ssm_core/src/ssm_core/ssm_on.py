#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import datetime
import rospy

def StrTimeStamped():
    return datetime.datetime.now().strftime("%Y_%m_%d_%H:%M:%S.%f   - ")

class onEntry():
    
    def __init__(self, logs={}, script=None):
        self._logs = logs
        self._script = script
    
    def execute(self, ud):
        if(rospy.get_param("ssm_enable_log", False) == True):
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
                
        if(rospy.get_param("ssm_enable_log", False) == True):
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
