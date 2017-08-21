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

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from alarm import Alarm
from account import User

from cobot_gui.res import R

class UiLogger:
    
    INFO     = QMessageBox.Information
    WARN     = QMessageBox.Warning
    CRITICAL = QMessageBox.Critical
    QUESTION = QMessageBox.Question
    
    @staticmethod
    def log(level, message):
        massage_box = QMessageBox()
        massage_box.setWindowFlags(Qt.FramelessWindowHint)
        massage_box.setIcon(level)
        massage_box.setText(message)
        return massage_box.exec_()
        
    @staticmethod
    def info(message):
        return UiLogger.log(UiLogger.INFO, message)
    
    @staticmethod
    def warn(message):
        return UiLogger.log(UiLogger.WARN, message)
        
    @staticmethod
    def err(message):
        return UiLogger.log(UiLogger.CRITICAL, message)
    
    @staticmethod
    def critical(message):
        return UiLogger.log(UiLogger.CRITICAL, message)
        
    @staticmethod
    def question(message):
        return UiLogger.log(UiLogger.QUESTION, message)

class Context(QObject):
    
    DEFAULT_LNG = 'en'
    
    AUTOMATIC = 1
    MANUAL    = 2
    
    viewManagerRequest = SIGNAL('viewManagerRequested')
    
    newUserConnectionRequest = SIGNAL('newUserConnectionRequested')
    newControlModeRequest    = SIGNAL('newControlModeRequested')
    newLanguageRequest       = SIGNAL('newLanguageRequested')
    
    defaultTrigger       = SIGNAL('defaultTriggered')
    alarmTrigger         = SIGNAL('alarmTriggered')
    emergencyStopRequest = SIGNAL('emergencyStopRequested')
    shutingdownRequest   = SIGNAL('shutingdownRequested')
    
    def __init__(self, parent):
        QObject.__init__(self, parent)
        
        self._parent       = parent
        self._user         = User()
        self._control_mode = self.AUTOMATIC
        self._lng          = self.DEFAULT_LNG
        
        self._default = 0
        self._emer    = False
        self._what    = ''
        
    def getParent(self):
        return self._parent
        
    def addViewManagerEventListner(self, listener):
        self.connect(self, self.viewManagerRequest, listener)
        
    def addUserEventListener(self, listener):
        self.connect(self, self.newUserConnectionRequest, listener)
        
    addUserConnectionEventListener = addUserEventListener
        
    def addControlModeEventListener(self, listener):
        self.connect(self, self.newControlModeRequest, listener)
        
    def addLanguageEventListner(self, listener):
        self.connect(self, self.newLanguageRequest, listener)
        
    def addDefaultEventListner(self, listener):
        self.connect(self, self.defaultTrigger, listener)
        
    def addAlarmEventListner(self, listener):
        self.connect(self, self.alarmTrigger, listener)
        
    def addEmergencyStopEventListner(self, listener):
        self.connect(self, self.emergencyStopRequest, listener)
        
    def addCloseEventListner(self, listener):
        self.connect(self, self.shutingdownRequest, listener)
        
    addShutingdownEventListner = addCloseEventListner
        
    def getR(self):
        return R
    
    def getLogger(self):
        return UiLogger
    
    def requestNewUserConnection(self, user):
        self._user = user
        self.emit(self.newUserConnectionRequest, user)
        
    switchUser = requestNewUserConnection
    
    def getUser(self):
        return self._user
    
    getUserInfo = getUser
    
    def requestDisplayView(self, view):
        self.emit(self.viewManagerRequest, view)
        
    def requestNewControlMode(self, ctrl):
        self._control_mode = ctrl
        self.emit(self.newControlModeRequest, ctrl)
        
    switchControlMode = requestNewControlMode
        
    def getControlMode(self):
        return self._control_mode
        
    def requestNewLanguage(self, lng):
        self._lng = lng
        self.emit(self.newLanguageRequest, lng)
        
    switchLanguage = requestNewLanguage
        
    def getLanguage(self):
        return self._lng
    
    def requestDefault(self, errcode, what=''):
        self._default = errcode
        self._what = what
        self.emit(self.defaultTrigger, self._default)
        
    def getDefault(self):
        return self._default
    
    def requestAlarm(self, level, what="???"):
        self.emit(self.alarmTrigger, Alarm(self, level, what))
        
    sendAlarm = requestAlarm
        
    def getAlarm(self):
        return self._alarm

    def resquestEmergencyStop(self, state, what=''):
        self._emer = state
        self._what = what
        self.emit(self.emergencyStopRequest, state)
    
    def inEmergencyStop(self):
        return self._emer
    
    def what(self):
        return self._what
    
    def requestShutdown(self):
        self.emit(self.shutingdownRequest)
    
