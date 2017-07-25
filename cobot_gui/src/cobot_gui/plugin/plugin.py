#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : plugin_installer.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

import rospy
from wrapper_plugin import WrapperPlugin
from cobot_gui.control_mode import ControlMode
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

