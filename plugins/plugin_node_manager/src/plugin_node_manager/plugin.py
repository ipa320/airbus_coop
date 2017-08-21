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
import time
import os
import re
import subprocess
import rosnode

from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from python_qt_binding import loadUi

from table_monitoring_nodes import TableMonitoringNodes
# from table_launch_nodes import TableLaunchNodes

from plugin_node_manager.res import R

from cobot_gui import Plugin, ControlMode

class PluginNodeManager(Plugin):
    
    def __init__(self, context):
        Plugin.__init__(self, context)
        
    def onCreate(self, param):
        
        # Extend the widget with all attributes and children from UI file
        loadUi(R.layouts.mainwindow, self)
        
        self.monitoring = TableMonitoringNodes(self)
        self.monitoring.onStart()
        
    def onPause(self):
        pass
    
    def onResume(self):
        pass
    
    def onControlModeChanged(self, mode):
        pass
        
    def onUserChanged(self, user_info):
        pass
    
    def onTranslate(self, lng):
        self.monitoring.translate(lng)
    
    def onEmergencyStop(self, state):
        pass
    
    def onDestroy(self):
        pass
    
#End of file
