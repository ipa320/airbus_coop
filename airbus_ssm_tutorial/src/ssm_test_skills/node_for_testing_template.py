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


import rospy

from airbus_ssm_core.msg import SimpleStateExecutorAction, SimpleStateExecutorResult
from std_msgs.msg import Empty
import actionlib


##
class SimpleStateActionServer():
    def __init__(self):
        self.server = actionlib.SimpleActionServer("/ssm/test_action_server", SimpleStateExecutorAction, self.execute, False)
        self.server.start()
        
    def execute(self, goal):
        for i in range(1,10):
            print(i)
            rospy.sleep(1)
            if(self.server.is_preempt_requested()):
                result = SimpleStateExecutorResult()
                result.outcome = "fail"
                self.server.set_preempted(result)
                rospy.loginfo("canceling goal")
                return
        
        result = SimpleStateExecutorResult()
        result.outcome = "next"
        self.server.set_succeeded(result)
        

    
if __name__ == '__main__':
    
    rospy.init_node('ssm_test_node', log_level=rospy.INFO)
    server = SimpleStateActionServer()
    rospy.spin()
    
        