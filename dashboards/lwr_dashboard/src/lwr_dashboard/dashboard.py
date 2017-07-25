#!/usr/bin/env python

## @package: robot_dashboard
## @version 1.0
## @author  Matignon Martin
## @date    Last modified 24/04/2015

import rospy

from lwr_driver.msg import RobotStatus

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber

from cobot_gui import Dashboard, DashboardPopup

from lwr_dashboard.res import R

class LWRPopup(DashboardPopup):
    
    def __init__(self, parent):
        DashboardPopup.__init__(self, parent)
        
    def onCreate(self, param):
        self.setRelativePosition(DashboardPopup.TopRight,
                                 DashboardPopup.BottomRight)
        
    def onDestroy(self):
        pass

class LWRDashboard(Dashboard):
    
    def __init__(self, context):
        Dashboard.__init__(self, context)
        
    def onCreate(self, param):
        
        self._ic_lwr_in_run = R.getPixmapById('icon_robot_running')
        self._ic_lwr_in_err = R.getPixmapById('icon_robot_error')
        
        self._lwr_status_label = QLabel()
        self._curr_pixmap = self._ic_lwr_in_err
        self._lwr_status_label.setPixmap(self._curr_pixmap.scaled(
                                          self.width(),
                                          self.height(),
                                          Qt.KeepAspectRatio,
                                          Qt.SmoothTransformation))
        
        self.getLayout().addWidget(self._lwr_status_label)
        
        self._robot_status = RobotStatus()
        self._in_timeout = False
        self._robot_status_sub = QAgiSubscriber(self,
                                                '/rsi_prog/status',
                                                 RobotStatus,
                                                 self._update_robot_status_,
                                                 self._robot_status_timeout_,
                                                 timeout = rospy.Duration(3),
                                                 max_rate = rospy.Rate(3))
    def _update_robot_status_(self, robot):
         
        self._in_timeout = False
         
        if robot.status != self._robot_status.status :
            
            self._robot_status = robot
             
            if self._robot_status.status < 0:
                self._curr_pixmap = self._ic_lwr_in_err
            else:
                self._curr_pixmap = self._ic_lwr_in_run
             
            self._lwr_status_label.setPixmap(self._curr_pixmap.scaled(
                                                  self.width(),
                                                  self.height(),
                                                  Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation
                                              ))
         
    def _robot_status_timeout_(self):
         
        if not self._in_timeout:
             
            self._robot_status = RobotStatus()
            self._lwr_status_label.setPixmap(self._ic_lwr_in_err.scaled(
                                                  self.width(),
                                                  self.height(),
                                                  Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation
                                              ))
            self._in_timeout = True
    
    def onRequestPopup(self):
        return None
        
    def onControlModeChanged(self):
        pass
    
    def onTranslate(self, lng):
        pass
    
    def onEmergencyStop(self, state):
        pass
    
    def onDestroy(self):
        pass
