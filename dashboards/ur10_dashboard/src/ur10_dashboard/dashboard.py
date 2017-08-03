#!/usr/bin/env python

## @package: robot_dashboard
## @version 1.0
## @author  Matignon Martin
## @date    Last modified 24/04/2015

import rospy

from std_msgs.msg import Int16

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber

from cobot_gui import Dashboard

from popup import UR10DashboardPopup

from ur10_dashboard.res import R

class UR10Dashboard(Dashboard):
    
    def __init__(self, context):
        Dashboard.__init__(self, context)
        
    def onCreate(self, param):
        
        self._ur10_status_label = QLabel()
        self._curr_pixmap = R.getPixmapById('icon_robot_error')
        self._ur10_status_label.setPixmap(self._curr_pixmap.scaled(
                                          self.width(),
                                          self.height(),
                                          Qt.KeepAspectRatio,
                                          Qt.SmoothTransformation))
        
        self.getLayout().addWidget(self._ur10_status_label)
        
        self._robot_status = Int16(-666)
        self._in_timeout = False
        self._robot_status_sub = QAgiSubscriber(self,
                                                '/ur10/state',
                                                 Int16,
                                                 self._update_robot_status_,
                                                 self._robot_status_timeout_,
                                                 timeout = rospy.Duration(3),
                                                 max_rate = rospy.Rate(3))
    def _update_robot_status_(self, status):
        
        self._in_timeout = False
        
        if status.data != self._robot_status.data :
            
            self._robot_status = status
            
            if status.data < 0:
                self._curr_pixmap = R.getPixmapById("icon_robot_error")
            else:
                self._curr_pixmap = R.getPixmapById("icon_robot_running")
            
            self._ur10_status_label.setPixmap(self._curr_pixmap.scaled(
                                                  self.width(),
                                                  self.height(),
                                                  Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation
                                              ))
        
    def _robot_status_timeout_(self):
        
        if not self._in_timeout:
            
            self._robot_status = Int16()
            self._ur10_status_label.setPixmap(R.getPixmapById("icon_robot_error").scaled(
                                                  self.width(),
                                                  self.height(),
                                                  Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation
                                              ))
            self._in_timeout = True
    
    def onRequestPopup(self):
        return None #UR10DashboardPopup(self)
        
    def onControlModeChanged(self):
        pass
    
    def onTranslate(self, lng):
        pass
    
    def onEmergencyStop(self, state):
        pass
    
    def onDestroy(self):
        pass
