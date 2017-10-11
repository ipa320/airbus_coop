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
import sys
from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi

from airbus_pyqt_extend.QtAgiGui import QAgiSilderButton
from airbus_cobot_gui.alarm import Alarm
from airbus_cobot_gui.emergency import EmergencyStopState
from airbus_cobot_gui.res import R
## @class ControlMode
## @brief Class for difine different control mode.
class ControlMode:
    
    AUTOMATIC = 1
    MANUAL    = 2
    
    TOSTR = {AUTOMATIC : 'Automatic',
             MANUAL    : 'Manual'}
    
    TOLEVEL = {'Automatic' : AUTOMATIC,
               'auto'      : AUTOMATIC,
               'Manual'    : MANUAL,
               'manu'      : MANUAL}

class ControlModeWidget(QWidget):
    
    def __init__(self, context):
        """! The constructor."""
        QWidget.__init__(self)
        
        self._context = context
        self._context.addLanguageEventListner(self.onTranslate)
        self._context.addAlarmEventListner(self.onAlarm)
        self._context.addEmergencyStopEventListner(self.onEmergencyStop)
        self._context.addCloseEventListner(self.onDestroy)
        
        self.setFixedSize(QSize(140,40))
        
        lng = self._context.getLanguage()
        
        self._slider_button = QAgiSilderButton(self,
                                            initial_state=True,
                                            on_label=R.values.strings.manu(lng),
                                            off_label=R.values.strings.auto(lng))
        self._slider_button.setFixedSize(QSize(140,40))
        self.connect(self._slider_button,
                     SIGNAL("statusChanged"),
                     self._switch_mode)
    
    def setDefaultMode(self, mode):
        
        if mode is ControlMode.AUTOMATIC:
            self._slider_button.setState(False)
        else:
            self._slider_button.setState(True)
            
        self._context.requestNewControlMode(mode)
        
    setControlMode = setDefaultMode
    
    def _switch_mode(self, status):
        
        if status == False:
            self._context.requestNewControlMode(self._context.AUTOMATIC)
        else:
            self._context.requestNewControlMode(ControlMode.MANUAL)
            
    def onTranslate(self, lng):
        self._slider_button.setOnText(R.values.strings.manu(lng))
        self._slider_button.setOffText(R.values.strings.auto(lng))
        
    def onAlarm(self, alarm):
        
        if alarm.level() >= Alarm.CRITICAL:
            if self._context.getControlMode() != ControlMode.MANUAL:
                self.setControlMode(ControlMode.MANUAL)
                self._context.sendAlarm(Alarm.WARNING,
                                        "Control mode switching automatic to manual mode !")
        
    def onEmergencyStop(self, emer):
        
        if emer == EmergencyStopState.LOCKED:
            if self._context.getControlMode() != ControlMode.MANUAL:
                self.setControlMode(ControlMode.MANUAL)
        
    def onDestroy(self):
        pass
        
if __name__ == "__main__":
    
    from airbus_cobot_gui.context import Context
    
    rospy.init_node('unittest_countrol_mode_ui')
    
    a = QApplication(sys.argv)
    
    utt_appli = QMainWindow()
    
    utt_appli.setCentralWidget(ControlModeWidget(Context(utt_appli)))
    
    utt_appli.show()
    a.exec_()
    
#End of file
