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
import random
from airbus_ssm_core import ssm_state



class Test1(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["success"], io_keys=["test"])
        
    def execution(self, ud):
        rospy.sleep(2)
        if("test" in ud):
            ud.test = int(ud.test) + 1
        else:
            ud.test = 0
        print("Test : " + str(ud.test))
        return "success"
    
class Test2(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["success","retry","next"])
        self.cpt_ = 0
        
    def execution(self, ud):
        rospy.sleep(1)
        choice = random.randint(1,3)
        self.cpt_ = self.cpt_ + choice
        if(self.cpt_ > 10):
            print("Test2 : success")
            return "success"
        elif(self.cpt_ > 5):
            print("Test2 : Retry")
            return "retry"
        elif(self.cpt_> 0):
            print("Test2 : Next")
            return "next"
        else:
            print("Test2 : success")
            return "success"
        
class Test3(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["success","failed"])
        
    def execution(self, ud):
        rospy.sleep(1)
        choice = random.randint(1,2)
        if(choice == 1):
            print("Test3 : success")
            return "success"
        else:
            print("Test3 : failed")
            return "failed"           
            

    
    
        
    

        
