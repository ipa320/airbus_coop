#!/usr/bin/env python
#
# Copyright 2015 Airbus
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import os
import time

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from pyqt_agi_extend import QtAgiCore

from airbus_cobot_gui.dashboard import Dashboard, DashboardPopup

from airbus_cobot_gui.res import R

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
        
