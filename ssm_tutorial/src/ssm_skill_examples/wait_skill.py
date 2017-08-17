#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
# Copyright 2015 Airbus
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
from ssm_core import ssm_state


class waitTime(ssm_state.ssmState):

    def __init__(self):
        ssm_state.ssmState.__init__(self, io_keys=['waitsec'], outcomes=['success'])
        
    def execution(self, ud):
        temps=int(ud.waitsec)
        r = rospy.Rate(100)
        cpt=0
        while (cpt < temps*100 or self.preempt_requested()):
            cpt = cpt + 1
            r.sleep()
            
        if(cpt==(temps*100)):
            return 'success'
        else:
            return 'preempt'
 
