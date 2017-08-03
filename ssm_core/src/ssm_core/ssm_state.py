#!/usr/bin/env python

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

