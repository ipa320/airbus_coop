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

from airbus_ssm_core.ssm_main import ssmMain
from std_msgs.msg import Empty
    
if __name__ == '__main__':
    
    rospy.init_node('ssm_main', log_level=rospy.INFO)
    SSM = ssmMain()
    if(rospy.get_param('ssm_autostart', False) == True):
       if(SSM._init_SSM()):
          SSM.start(Empty)
    rospy.spin()
    
