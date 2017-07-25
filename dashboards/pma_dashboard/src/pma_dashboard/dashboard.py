#!/usr/bin/env python

## @package: robot_dashboard
## @version 1.0
## @author  Matignon Martin
## @date    Last modified 24/04/2015

import os
import rospy

from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber

from cobot_gui import Dashboard
from popup import PMADashboardPopup

from pma_dashboard.res import R

class PMADashboard(Dashboard):
    
    def __init__(self, context):
        Dashboard.__init__(self, context)
        
    def onCreate(self, param):
        
        self._pma_status_label = QLabel()
        
        self._px_curr_status = R.getPixmapById("pma_error")
        
        self._pma_status_label.setPixmap(self._px_curr_status.scaled(
                                          self.width(),
                                          self.height(),
                                          Qt.KeepAspectRatio,
                                          Qt.SmoothTransformation))
        
        self.getLayout().addWidget(self._pma_status_label)
        
    def _update_pma1_status_(self, status):
        pass
        
    def _pma1_status_timeout_(self):
        pass
        
    def resizeEvent(self, event):
        pass
        
    def onRequestPopup(self):
        return PMADashboardPopup(self)
        
    def onControlModeChanged(self, mode):
        pass
    
    def onTranslate(self, lng):
        pass
    
    def onEmergencyStop(self, state):
        pass
    
    def onDestroy(self):
        pass
    
    
# my_unittest.shutdown()