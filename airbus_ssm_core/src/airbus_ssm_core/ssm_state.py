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
import actionlib
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
            if(self._datamodel[data] != ""):
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
        rospy.sleep(2)
        ud.skill = "Empty"
        return "next"
    
class FunctionState(ssmState):
    '''
    Execute a simple function
    '''
    def __init__(self, function, f_args=[], f_kwargs=[],outcomes=[], io_keys=[]):
        ssmState.__init__(self, outcomes, io_keys)
        self._f = function
        self._args = f_args
        self._kwargs = f_kwargs
        
    def execution(self, ud):
        return self._f(ud, *self._args, **self._kwargs)       
    
class SubscriberState(FunctionState):
    '''
    Subscribe to a topic and wait to receive a message to execute a function
    The function should be define that way f(msg, ud, *args, **kwargs)
    '''
    def __init__(self, topic_name, message_type, callback, f_args=[], f_kwargs=[],outcomes=[], io_keys=[], timeout_ = 60, timeout_outcome = "preempt"):
        FunctionState.__init__(self, function, f_args, f_kwargs, outcomes, io_keys)
        self._to = timeout_
        self._to_outcome = timeout_outcome
        self._in_execution = False
        self._function_executed = False
        self._function_outcome = ''
        self._subscriber = None
        self._local_userdata = None
        self.register_outcomes([timeout_outcome])
        self._subscriber = rospy.Subscriber(topic_name, message_type, cb_wrapper, queue_size=1)
        
    def onInit(self): ##NOT USE
        self._subscriber = rospy.Subscriber(topic_name, message_type, cb_wrapper, queue_size=1)
    
    def cb_wrapper(self, msg):
        if(self._in_execution):
            self._function_outcome = self._f(msg, self._local_userdata, *self._args, **self._kwargs)
            self._function_executed = True      
    
    def wait_for_message(self):
        to_ = rospy.Time.now() + self._to
        while not self._callback_return:
            if self.preempt_requested() == True:
                self.service_preempt()
                return "preempt"
            if rospy.Time.now() < to_:
                rospy.logwarn("'%s timed out : it will return the following outcome '%s'"
                             %(self.__class__.__name__,self._to_outcome))
                return self._to_outcome
            else:
                rospy.sleep(0.001)

        return  self._function_outcome
    
    def execution(self, ud):
        self._in_execution = True
        self._function_executed = False
        self._local_userdata = ud
        outcome = self.wait_for_message()
        ud = self._local_userdata
        self._function_executed = False
        self._in_execution = False
        return outcome
    
    def onDestroy(self): ##NOT USE
        self._subscriber.unregister()
        
class ServiceState(ssmState):
    '''
    Setup a message to call a service, wait for the answer and analyse the answer
    You have to implement two function : 
    
    '''
    def __init__(self, service_name, service_type, outcomes=[], io_keys=[], timeout_ = 60, timeout_outcome = "preempt"):
        ssmState.__init__(self, outcomes, io_keys)
        self._to = timeout_
        self._to_outcome = timeout_outcome
        self._serv_name = service_name
        self._serv_type = service_type
        self.register_outcomes([timeout_outcome])
    
    def setup_request(self, ud):
        raise NotImplementedError
    
    def analyse_answer(self, ud, service_answer):
        raise NotImplementedError
    
    def wait_for_service(self, service_request):
        to_ = rospy.time(),now() + self._to
        service_answer = None
        while service_answer == None:
            rospy.sleep(0.01)
            if self.preempt_requested() == True:
                self.service_preempt()
                return "preempt"
            if rospy.Time.now() < to_:
                rospy.logwarn("'%s timed out : it will return the following outcome '%s'"
                             %(self.__class__.__name__,self._to_outcome))
                return "timeout"
            else:
                try:
                    rospy.wait_for_service(self._serv_name,1)
                except Exception as e:
                    continue
                try:
                    service_caller = rospy.ServiceProxy(self._serv_name, self._serv_type)
                    service_answer = service_caller(service_request)
                    return service_answer
                except Exception as e:
                    continue

    
    def execution(self, ud):
        service_request = self.setup(ud)
        service_answer = self.wait_for_service(service_request)
        if service_answer == "preempt":
            return "preempt"
        elif service_answer == "timeout":
            return self._to_outcome
        else:
            return self.analysis(ud, service_answer)
        
class ActionClientState(ssmState):
    '''
    
    
    '''
    def __init__(self, action_server_name, action_type, outcomes=[], io_keys=[], timeout_server = 10, timeout_server_outcome = "preempt", timeout_action = 120, timeout_action_outcome = "preempt"):
        ssmState.__init__(self, outcomes, io_keys)
        self._tosv = timeout_server
        self._tosv_outcome = timeout_server_outcome
        self._toac = timeout_action
        self._toac_outcome = timeout_action_outcome
        self._as_name = action_server_name
        self._action_type = action_type
        self._local_userdata = None
        self._done = False
        
        self.ac_client = None
        
        self.register_outcomes([timeout_server_outcome, timeout_action_outcome])
    
    def setup_goal(self, ud):
        raise NotImplementedError
    
    def analyse_feedback(self, ud, feedback_msg):
        pass
    
    def analyse_result(self, ud, result):
        raise NotImplementedError
    
    def done_cb(self):
        self._done = True
    
    def feedback_wrapper(self, msg):
        self.analyse_feedback(self._local_userdata, msg)
    
    def wait_for_server(self):
        to_ = rospy.time(),now() + self._tosv
        server_connected = False
        while server_connected == False:
            rospy.sleep(0.01)
            if self.preempt_requested() == True:
                self.service_preempt()
                return "preempt"
            if rospy.Time.now() < to_:
                rospy.logwarn("'%s timed out : it will return the following outcome '%s'"
                             %(self.__class__.__name__,self._to_outcome))
                return "timeout"
            else:
                server_connected = self.ac_client.wait_for_server(rospy.Duration(0.1))
                
    def wait_for_result(self):
        to_ = rospy.time(),now() + self._toac
        while self._done == False:
            if self.preempt_requested() == True:
                self.service_preempt()
                self.ac_client.cancel_goal()
                return "preempt"
            if rospy.Time.now() < to_:
                rospy.logwarn("'%s connection to server timed out : it will return the following outcome '%s'"
                             %(self.__class__.__name__,self._to_outcome))
                self.ac_client.cancel_goal()
                return "timeout"
            else:
                rospy.sleep(0.01)

        
    def execution(self, ud):
        self._local_userdata = ud
        goal = self.setup_goal(ud)
        self.ac_client = actionlib.SimpleActionClient(self._as_name, self._action_type)
        connection = self.wait_for_server()
        if connection == "preempt":
            return "preempt"
        elif connection == "timeout":
            return self._tosv_outcome
        else:
            pass
        self._done = False
        self.ac_client.send_goal(goal, done_cb = self.done_cb, active_cb=None, feedback_cb = self.feedback_wrapper)
        result = self.wait_for_result()
        if result == "preempt":
            return "preempt"
        elif result == "timeout":
            return self._toac_outcome
        else:
            pass
        ud = self._local_userdata
        return self.analyse_result(ud, self.ac_client.get_result())
    
