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
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D

from cobot_gui import plugin, ControlMode, EmergencyStopState

from ur10_effector_plugin.res import R
from PyQt4.Qt import QLabel

class ur10effectorPlugin(plugin.Plugin):
    
    def __init__(self, context):
        """ Plugin constructor.
        It would be better not to change anything here.
        """
        plugin.Plugin.__init__(self, context)
    def onCreate(self, param):
        self.Triger_Publisher=rospy.Publisher("/ur10/actionner", Empty, queue_size=1)
        self.Camera_Publisher=rospy.Publisher("/camera_ur10/trigger", Empty, queue_size=1)
        self.Pose_Suscriber=rospy.Subscriber("/camera_ur10/pose", Pose2D, self.pose_cb)
        loadUi(R.layouts.mainwindow, self)
        self.connect(self.TrigerButton, SIGNAL("clicked()"), self.trigger_vissage)
        self.connect(self.CameraButton, SIGNAL("clicked()"), self.trigger_camera)
        
    
    def onPause(self):
        self.Triger_Publisher.unregister()
        self.Triger_Publisher = None
        self.Camera_Publisher.unregister()
        self.Camera_Publisher = None
    
    def onResume(self):
        if self.Triger_Publisher is None:
            self.Triger_Publisher=rospy.Publisher("/ur10/actionner", Empty, queue_size=1)
        if self.Camera_Publisher is None:
            self.Camera_Publisher=rospy.Publisher("/camera_ur10/trigger", Empty, queue_size=1)
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
        
    def trigger_vissage(self):
        self.Triger_Publisher.publish()
        
    def trigger_camera(self):
        self.Camera_Publisher.publish()
    
    def pose_cb(self, msg):
        if msg.theta==1.0:
            self.Pose_label.setText(str("x=%f y=%f" % (msg.x, msg.y) ))
        else:
            self.Pose_label.setText("trou=absent")

if __name__ == "__main__":
    
    import sys
    
    rospy.init_node("ur10_effector_plugin_node")
    
    a = QApplication(sys.argv)
    
    window = plugin.getStandAloneInstance("ur10_effector_plugin", ur10effectorPlugin, "en")
    window.setWindowTitle("ur10effectorPlugin")
    window.show()
    a.exec_()
