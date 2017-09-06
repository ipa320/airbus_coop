#!/usr/bin/env python
#
# Copyright 2015 Airbus
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


import traceback
import rospy
import smach

__all__ = ['State','CBState']

class ssmState(smach.State):
    
    def __init__(self, outcomes=[], io_keys=[]):
        io_keys.append("logfile")
        io_keys.append("skill")
        smach.State.__init__(self, outcomes, io_keys=io_keys)
        self.register_outcomes(["preempt"])
        self._datamodel = {}
        self._onEntry = None
        self._onExit  = None
    
    def execute(self, ud):
        
        if self.preempt_requested():
            self.service_preempt()
            return "preempt"
        
        ## Copy the datamodel's value into the userData
        for data in self._datamodel:
            ud[data] = self._datamodel[data]
        
        ## Do the <onentry>
        if(self._onEntry is not None):
            try:
                self._onEntry.execute(ud)
            except Exception as ex:
                rospy.logerr('%s::onEntry::execute() raised | %s'
                             %(self.__class__.__name__,str(ex)))
                return "preempt"
        
        ## Execution    
        try:
            outcome = self.execution(ud)
        except Exception as ex:
            rospy.logerr('%s::execute() raised | %s'
                             %(self.__class__.__name__,str(ex)))
            return "preempt"
        finally:
            if self.preempt_requested():
                self.service_preempt()
                return "preempt"
        
        ## Do the <onexit>    
        if(self._onExit is not None):
            try:
                outcome = self._onExit.execute(ud, outcome)
            except Exception as ex:
                rospy.logerr('%s::onExit::execute() raised | %s'
                             %(self.__class__.__name__,str(ex)))
                return "preempt"
        
        return outcome
    
    def execution(self, ud):
        """Called when executing a state.
        In the base class this raises a NotImplementedError.

        @type ud: L{UserData} structure
        @param ud: Userdata for the scope in which this state is executing
        """
        raise NotImplementedError()
    
class EmptyState(ssmState):
    def __init__(self):
        ssmState.__init__(self,outcomes=["next"])
        
    def execution(self, ud):
        ud.skill = "Empty"
        return "next"

