#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : control_mode.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

## @package: control_mode
##
## @version 1.0
## @author  Matignon Martin
## @date    Last modified 24/09/2014

import rospy
import os
import sys
from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiGui import QAgiSilderButton
from cobot_gui.alarm import Alarm
from cobot_gui.emergency import EmergencyStopState
from cobot_gui.res import R
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
    
    from cobot_gui.context import Context
    
    rospy.init_node('unittest_countrol_mode_ui')
    
    a = QApplication(sys.argv)
    
    utt_appli = QMainWindow()
    
    utt_appli.setCentralWidget(ControlModeWidget(Context(utt_appli)))
    
    utt_appli.show()
    a.exec_()
    
#End of file