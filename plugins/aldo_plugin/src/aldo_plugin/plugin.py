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

from aldo_plugin.res import R
from rviz_robot import RVizRobot
from joint_control_ui import LeftJointControlUi,RightJointControlUi
from cartesian_control_ui import CartesianControlUi

class aldoPlugin(plugin.Plugin):
    
    INDEX_TAB_FOR_LEFT_JOINT_CTRL  = 0
    INDEX_TAB_FOR_RIGHT_JOINT_CTRL = 1
    INDEX_TAB_FOR_CART_CTRL        = 2
    
    
    
    def __init__(self, context):
        """ Plugin constructor.
        It would be better not to change anything here.
        """
        plugin.Plugin.__init__(self, context)
        
    def onCreate(self, param):

        # Load ui file
        loadUi(R.layouts.mainwindow, self)
        
        self._robot_ui = RVizRobot(self)
        robot_widget = self._robot_ui.get_widget()
        self.robot_view.setWidget(robot_widget)
        
        self.left_joint_control = LeftJointControlUi()      
        self.motion_tab.addTab(self.left_joint_control, "Joint Left Arm")
        self.right_joint_control = RightJointControlUi()
        self.motion_tab.addTab(self.right_joint_control, "Joint Right Arm")
        self.cartesian_control = CartesianControlUi()
        self.motion_tab.addTab(self.cartesian_control, "Cartesian Control")
        
        self.connect(self.motion_tab, SIGNAL("currentChanged(int)"),
                     self.slot_switch_motion_type)
        
    def slot_switch_motion_type(self, index):
        
        self.left_joint_control.unregister()
        self.right_joint_control.unregister()
        self.cartesian_control.unregister()
         
        if index == self.INDEX_TAB_FOR_LEFT_JOINT_CTRL:
            self.left_joint_control.subscribe()
        elif index == self.INDEX_TAB_FOR_RIGHT_JOINT_CTRL:
            self.right_joint_control.subscribe()
        elif index == self.INDEX_TAB_FOR_CART_CTRL:
            self.cartesian_control.subscribe()
        
    
    def onPause(self):
        self.left_joint_control.unregister()
        self.right_joint_control.unregister()
        self.cartesian_control.unregister()
        
    
    def onResume(self):
        index = self.motion_tab.currentIndex()
        if index == self.INDEX_TAB_FOR_LEFT_JOINT_CTRL:
            self.left_joint_control.subscribe()
        elif index == self.INDEX_TAB_FOR_RIGHT_JOINT_CTRL:
            self.right_joint_control.subscribe()
        elif index == self.INDEX_TAB_FOR_CART_CTRL:
            self.cartesian_control.subscribe()
        
    
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
        pass
        
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
        pass
    
    def onTranslate(self, lng):
        """ This method is called when the user change the language.
        You can retranslate your plugin here.
        """
        # Exemple:
        pass
    
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
    
    rospy.init_node("aldo_plugin_node")
    
    a = QApplication(sys.argv)
    
    window = plugin.getStandAloneInstance("aldo_plugin", aldoPlugin, "en")
    window.setWindowTitle("aldoPlugin")
    window.show()
    a.exec_()
