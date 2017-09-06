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
from wrapper_plugin import WrapperPlugin
from airbus_cobot_gui.control_mode import ControlMode
from PyQt4.Qt import QObject

## @package: plugin
##
## @version 4.0
## @author  Matignon Martin
## @date    Last modified 28/02/2014

## @class Plugin
## @brief Plugin interface..
class Plugin(WrapperPlugin):
    
    def __init__(self, context):
        WrapperPlugin.__init__(self, context)
        
    def onCreate(self, param):
        self.logWarn("Plugin %s.onCreate not implemented !"%self.getPluginName())
    
    def onPause(self):
        self.logWarn("Plugin %s.onPause not implemented !"%self.getPluginName())
    
    def onResume(self):
        self.logWarn("Plugin %s.onResume not implemented !"%self.getPluginName())
    
    def onControlModeChanged(self, mode):
        # Default rules, you can custom this method to set your rules
        if mode == ControlMode.AUTOMATIC:
            self.setEnabled(False)
        else:
            self.setEnabled(True)
        
    def onUserChanged(self, user_info):
        self.logWarn("Plugin %s.onUserChanged not implemented !"%self.getPluginName())
    
    def onTranslate(self, lng):
        self.logWarn("Plugin %s.onTranslate not implemented !"%self.getPluginName())
    
    def onEmergencyStop(self, state):
        self.tryToPause()
    
    def onDestroy(self):
        self.tryToPause()
    
    
#End of file

