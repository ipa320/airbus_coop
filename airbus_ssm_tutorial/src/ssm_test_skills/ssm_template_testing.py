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


from airbus_ssm_core.ssm_state import ssmState, ssmFunctionState, ssmSubscriberState, ssmServiceState, ssmActionClientState, ssmSimpleActionClientState
from std_msgs.msg import Empty
from roscpp.srv import GetLoggers 
from airbus_ssm_core.msg import SimpleStateExecutorAction, SimpleStateExecutorGoal
import json
    
class EmptyState(ssmState):
    def __init__(self):
        ssmState.__init__(self,outcomes=["next"])
        
    def execution(self, ud):
        rospy.sleep(2)
        ud.skill = "Empty"
        return "next"
##    
def EmptyFunction(userdata, param1=0, param2=2, param3="test"):
    print(userdata.data)
    if(param1 > param2):
        print(param3)
    else:
        print("failed")
    return "next"

class EmptyFunctionState(ssmFunctionState):
    def __init__(self):
        ssmFunctionState.__init__(self, EmptyFunction, (1,-1),{"param3":"TESTING"}, io_keys=["data"], outcomes = ["next"])
##    

##
class EmptySubscriberState(ssmSubscriberState):
    def __init__(self):
        ssmSubscriberState.__init__(self, "/ssm/TestEmptySubscriber", Empty, (1,10),{"param3":"TESTING"}, outcomes=["next"], io_keys=["data"], timeout_ = 60, timeout_outcome = "preempt")
        
    def callback(self, msg, userdata, param1=0, param2=0, param3="Test"):
        print(userdata.data)
        if(param1 > param2):
            print(param3)
        else:
            print("failed")
        return "next"
##

##    
class EmptyServiceState(ssmServiceState):
    def __init__(self):
        ssmServiceState.__init__(self, "/rosout/get_loggers", GetLoggers, outcomes=["next"], io_keys=["data"], timeout_ = 60, timeout_outcome = "preempt")
    
    def setup_request(self, ud):
        return GetLoggers()._request_class()
    
    def analyse_answer(self, ud, service_answer):
        print(service_answer)
        return "next"
##

##    
class EmptyActionState(ssmActionClientState):
    def __init__(self):
        ssmActionClientState.__init__(self, "/ssm/test_action_server", SimpleStateExecutorAction, outcomes=["next", "fail"], io_keys=["data"], timeout_server = 10, timeout_server_outcome = "preempt", timeout_action = 15, timeout_action_outcome = "preempt")

    def setup_goal(self, ud):
        dict = {}
        _keys = self.get_registered_input_keys()
        if(len(_keys) > 2):
            for key in _keys:
                 if(key != "skill" and key != "logfile"):
                     dict[key] = ud[key]
        
        goal = SimpleStateExecutorGoal()
        goal.JSON_data = json.dumps(dict)
        return 
    
    def analyse_feedback(self, ud, feedback_msg):
        pass
    
    def analyse_result(self, ud, result):
        print(result)
        return result.outcome
##  
    
        