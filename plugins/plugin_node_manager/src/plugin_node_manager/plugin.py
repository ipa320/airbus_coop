#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : node_manager.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
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
