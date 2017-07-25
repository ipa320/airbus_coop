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

