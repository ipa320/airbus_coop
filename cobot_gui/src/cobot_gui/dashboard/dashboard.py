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


import rospy
from wrapper_dashboard import WrapperDashboard

## @package: plugin
##
## @version 4.0
## @author  Matignon Martin
## @date    Last modified 28/02/2014

## @class Plugin
## @brief Plugin interface..
class Dashboard(WrapperDashboard):
    
    def __init__(self, context):
        WrapperDashboard.__init__(self, context)
        
    def onCreate(self, param):
        self.logWarn("Dashboard %s.onCreate not implemented !"%self.getName())
    
    def onControlModeChanged(self, mode):
        self.logWarn("Dashboard %s.onControlModeChanged not implemented !"%self.getName())
        
    def onUserChanged(self, user):
        # Default rules, you can custom this method to set your rules
        if user.getUserPrivilege() >= self.getAccessRights():
            self.setPopupEnabled(True)
        else:
            self.setPopupEnabled(False)
    
    def onTranslate(self, lng):
        self.logWarn("Dashboard %s.onTranslate not implemented !"%self.getName())
    
    def onEmergencyStop(self, state):
        self.logWarn("Dashboard %s.onEmergencyStop not implemented !"%self.getName())
    
    def onDestroy(self):
        self.logWarn("Dashboard %s.onDestroy not implemented !"%self.getName())
    
#End of file

