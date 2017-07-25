#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : ssm_plugin
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
import time

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from pyqt_agi_extend import QtAgiCore

from cobot_gui.dashboard import Dashboard, DashboardPopup

from cobot_gui.res import R

class CalendarPopup(DashboardPopup):
    def __init__(self, parent):
        DashboardPopup.__init__(self, parent)
        
        self.setRelativePosition(DashboardPopup.TopRight,
                                 DashboardPopup.BottomRight)
        
    def onCreate(self, param):
        
        grid_layout = QGridLayout(self)
        grid_layout.setSpacing(0)
        grid_layout.setContentsMargins(0, 0, 0, 0)
        
        calendar = QCalendarWidget()
        
        grid_layout.addWidget(calendar,0,0,1,1)
    
    def onDestroy(self):
        pass

class Timestamp(Dashboard):
    
    def __init__(self, context):
        Dashboard.__init__(self, context)
        
    def onCreate(self, param):
        
        self._time_label = QLabel()
        self._time_label.setStyleSheet(R.values.styles.text)
        self._time_label.setObjectName("clock")
        self._time_label.setAlignment(Qt.AlignCenter)
        
        self.getLayout().addWidget(self._time_label)
        
        clk_time = time.strftime("%H:%M", time.localtime(time.time()))
        self._time_label.setText(clk_time)
        
        self._t_datetime = QTimer(self)
        self.connect(self._t_datetime, SIGNAL("timeout()"), self._update_time)
        self._t_datetime.start(3600)
        
    def _update_time(self):
        
        clk_time = time.strftime("%H:%M", time.localtime(time.time()))
        self._time_label.setText(clk_time)
        
    def resizeEvent(self,event):
        pass
#         self.resize(self._time_label.width(), self._time_label.height())
        
    def onRequestPopup(self):
        return CalendarPopup(self)
        
    def onControlModeChanged(self):
        pass
    
    def onTranslate(self, lng):
        pass
    
    def onEmergencyStop(self, state):
        pass
    
    def onDestroy(self):
        self._t_datetime.stop()
        