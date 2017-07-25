#!/usr/bin/env python

## @package: robot_dashboard
## @version 1.0
## @author  Matignon Martin
## @date    Last modified 24/04/2015

import rospy

from industrial_msgs.msg import RobotStatus, TriState

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber

from pyqt_agi_extend import QtAgiCore
from cobot_gui import dashboard

from default_dialog import RobotDefaultHandleDialog
from popup import IIWADashboardPopup

from iiwa_dashboard.res import R

class IIWADashboard(dashboard.Dashboard):
    
    def __init__(self, context):
        dashboard.Dashboard.__init__(self, context)
        
    def onCreate(self, param):
        
        self._iiwa_status_label = QLabel()
        self._curr_pixmap = R.getPixmapById('icon_robot_error')
        self._iiwa_status_label.setPixmap(self._curr_pixmap.scaled(
                                          self.width(),
                                          self.height(),
                                          Qt.KeepAspectRatio,
                                          Qt.SmoothTransformation))
        
        self.getLayout().addWidget(self._iiwa_status_label)
        
        self._robot_status = RobotStatus()
        self._in_timeout = False
        self._robot_status_sub = QAgiSubscriber(self,
                                                '/iiwa/status',
                                                 RobotStatus,
                                                 self._update_robot_status_,
                                                 self._robot_status_timeout_,
                                                 timeout = rospy.Duration(3),
                                                 max_rate = rospy.Rate(3))
    def _update_robot_status_(self, status):
        
        self._in_timeout = False
        
        if self.diff_status(status, self._robot_status) :
            
            self._robot_status.drives_powered.val = status.drives_powered.val
            self._robot_status.e_stopped.val      = status.e_stopped.val
            self._robot_status.error_code         = status.error_code
            
            if status.error_code < 0:
                rdf = RobotDefaultHandleDialog(self, status.error_code)
                rdf.show()
            
            if status.drives_powered.val == TriState.TRUE:
                
                if status.e_stopped.val == TriState.TRUE :
                    self._curr_pixmap = R.getPixmapById("icon_robot_e_stopped")
                elif status.error_code < 0:
                    if status.error_code == -99:
                        self._curr_pixmap = R.getPixmapById("icon_robot_collision")
                    else:
                        self._curr_pixmap = R.getPixmapById("icon_robot_error")
                else:
                    self._curr_pixmap = R.getPixmapById("icon_robot_running")
            else:
                self._curr_pixmap = R.getPixmapById("icon_robot_error")
            
            self._iiwa_status_label.setPixmap(self._curr_pixmap.scaled(
                                                  self.width(),
                                                  self.height(),
                                                  Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation
                                              ))
        
    def _robot_status_timeout_(self):
        
        if not self._in_timeout:
            
            self._robot_status = RobotStatus()
            self._iiwa_status_label.setPixmap(R.getPixmapById("icon_robot_error").scaled(
                                                  self.width(),
                                                  self.height(),
                                                  Qt.KeepAspectRatio,
                                                  Qt.SmoothTransformation
                                              ))
            self._in_timeout = True
        
    def diff_status(self, curr, last):
        
        if curr.drives_powered.val != last.drives_powered.val or \
           curr.e_stopped.val      != last.e_stopped.val or \
           curr.error_code         != last.error_code :
            return True
        else:
            return False
        
    def onRequestPopup(self):
        return IIWADashboardPopup(self)
        
    def onControlModeChanged(self, mode):
        pass
    
    def onTranslate(self, lng):
        pass
    
    def onEmergencyStop(self, state):
        pass
    
    def onDestroy(self):
        pass
    
    
if __name__ == "__main__":
    
    import sys
    from cobot_gui import dashboard
     
    rospy.init_node("iiwa_dashboard_node")
     
    a = QApplication(sys.argv)
     
    window = dashboard.getStandAloneInstance("iiwa_dashboard", IIWADashboard)
    window.setWindowTitle("IIWA dashboard")
    window.show()
    a.exec_()
