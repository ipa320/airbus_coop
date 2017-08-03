#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : plugin.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

import rospy
import os

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from cobot_gui import plugin, ControlMode, EmergencyStopState

from avancement_station_plugin.res import R

class AvancementStationPlugin(plugin.Plugin):
    
    def __init__(self, context):
        """ Plugin constructor.
        It would be better not to change anything here.
        """
        plugin.Plugin.__init__(self, context)
        
    def onCreate(self, param):
        """ Load and sets your GUI components and variables declaration.
        @param param: C{Parameters}
        
        onCreate set the parameters from the plugin_descriptor.xml (params element) from the cobot_gui config ($MY_COBOT_GUI.conf)
        
        Example:
        # Parameters provider
        my_param = param.getParam($MY_PARAM_NAME, $MY_DEFAULT_VALUE)
        
        # Ros topics declaration
        self._my_sub = None # Sets subcriber instance in onResume()
        self._my_pub = None # Sets publisher instance in onResume()
        """
        # Load ui file
        loadUi(R.layouts.mainwindow, self)
        
        # Set default style sheet
        self.title_label.setStyleSheet(R.values.styles.hello)
        
    
    def onPause(self):
        """ This method is called when the plugin go to background. (when another plugin go foreground)
        You can stop subcribers and connections
        
        Exemple:
        
        if self._my_sub is not None:
            self._my_sub.unregister()
            self._my_sub = None
            
        if self._my_pub is not None:
            self._my_pub.unregister()
            self._my_pub = None
        
        """
    
    def onResume(self):
        """ This method is called when the plugin go to foreground.
        You can start/restart subcribers and connections
        
        Exemple:
        
        if self._my_sub is None:
            self._my_sub = rospy.Subcriber(...)
            
        if  self._my_pub is None:
            self._my_pub = rospy.Publisher(...)
        """
        pass
    
    def onControlModeChanged(self, mode):
        """ This method is called when the user change the control mode.
        @param mode: C{ControlMode}
        
        You can write rules when the control mode changes.
        
        Example:
        NB: import needed (from cobot_gui import ControlMode)
        
        if mode == ControlMode.AUTOMATIC:
            # Do somethings
        elif mode == ControlMode.MANUAL:
            # Do somethings
        """
        
    def onUserChanged(self, user):
        """ This method is called when a new user logged-in.
        @param user_info: C{User}
        
        You can define different behaviors according to the type of user.
        
        Example:
        NB: import needed (from cobot_gui import UserPrivilege)
        
        if user.getUserPrivilege() == UserPrivilege.DEVELOPER:
            # Do some rules
        else:
            # Do some rules
        """
    
    def onTranslate(self, lng):
        """ This method is called when the user change the language.
        You can retranslate your plugin here.
        """
        # Exemple:
        self.title_label.setText(R.values.strings.hello(lng))
    
    def onEmergencyStop(self, state):
        """ This method is called when the emergency stop state changed.
        You can make emergency action here.
        """
        # Default emergency routine :
        if state == EmergencyStopState.LOCKED:
            self.onPause()
        elif state == EmergencyStopState.UNLOCKED:
            self.onResume()
    
    def onDestroy(self):
        """ This method is called when cobot_gui closes.
        You can free memory and disconnects topics
        """
        # Default exit routine :
        self.onPause()
        
if __name__ == "__main__":
    
    import sys
    
    rospy.init_node("avancement_station_plugin_node")
    
    a = QApplication(sys.argv)
    
    window = plugin.getStandAloneInstance("avancement_station_plugin", AvancementStationPlugin, "en")
    window.setWindowTitle("AvancementStationPlugin")
    window.show()
    a.exec_()
