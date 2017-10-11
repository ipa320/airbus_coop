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
import time

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi

from airbus_cobot_gui.res import R

## @class Alarm
## @brief Class for alarm structure.
class Alarm:
    
    NONE      = 0
    WARNING   = 1
    ERROR     = 2
    CRITICAL  = 3
    FATAL     = 4
    
    def __init__(self, context, level, what):
        self._context = context
        self._level = level
        self._what  = what
    
    def level(self):
        return self._level
    
    def what(self):
        return self._what
    
    def acquit(self):
        self._context.acquitAlarm()
    
class Incremental(QThread):
    
    def __init__(self):
        QThread.__init__(self)
        
        self._start_pos = 30
        self._end_pos   = 300
        self._step      = 1
        
    def init(self, start_pos, end_pos, step, speed = 0.0005):
        
        self._start_pos = start_pos
        self._end_pos   = end_pos
        self._step      = step
        
    def run(self):
        
        for i in range(self._start_pos, self._end_pos, self._step):
            self.emit(SIGNAL('valueChanged(int)'),i)
            time.sleep(0.0005)
            
            
class AlarmWidget(QWidget):
    
    def __init__(self, parent, alarm):
        QWidget.__init__(self)
        
        self._parent = parent
        self._alarm = alarm
        self._item  = QListWidgetItem('')
        
        loadUi(R.layouts.alarm_widget, self)
        
        self.connect(self.acquit_button, SIGNAL('clicked()'), self.onAcquit)
        
        self.what.setText(alarm.what())
        self._item.setSizeHint(self.sizeHint())
        
    def getItem(self):
        return self._item
        
    def onAcquit(self):
        self._parent.onRemoveAlarm(self.getItem())
        
class AlarmManagerWidget(QWidget):
    
    OPENED = True
    CLOSED = False
    MINIMUM_HEIGHT = 80
    MAXIMUM_HEIGHT = 350
    
    def __init__(self, context):
        QWidget.__init__(self)
        
        loadUi(R.layouts.alarm_listview, self)
        
        self.setFixedHeight(self.MINIMUM_HEIGHT)
        self.alarm_listview.setSpacing(5)
        
        self._context = context
        self._parent  = context.getParent()
        self._status  = self.CLOSED
        self._lng     = context.getLanguage()
        
        self._incremental = Incremental()
        self.connect(self._incremental, SIGNAL('valueChanged(int)'), self.setFixedHeight)
        self.connect(self.drawable_button, SIGNAL('clicked()'), self.switchDrawable)
        
        self._parent.alarm_label.setPixmap(R.getPixmapById("ico_alarm").scaled(
                           75,75,
                           Qt.KeepAspectRatio,Qt.SmoothTransformation))
        
        self._context.addAlarmEventListner(self.onAppendAlarm)
        self._context.addLanguageEventListner(self.onTranslate)
        
        self._parent.alarm_label.setVisible(False)
        self.setVisible(False)
    
    def getDrawableHeight(self):
        
        h = self.MINIMUM_HEIGHT+self.drawable_button.height()
        
        if self.alarm_listview.count() > 1:
            h = ((self.alarm_listview.count()+1)*40)+20
            if h > self.MAXIMUM_HEIGHT:
                h = self.MAXIMUM_HEIGHT
        
        return h
    
    def openDrawable(self):
        self._incremental.init(self.MINIMUM_HEIGHT, self.getDrawableHeight(), 1)
        self._incremental.start()
        self._status = self.OPENED
            
    def closeDrawable(self):
        self._incremental.init(self.getDrawableHeight(), self.MINIMUM_HEIGHT, -1)
        self._incremental.start()
        self._status = self.CLOSED
        
    def switchDrawable(self):
        
        if self._status == self.CLOSED:
            self.openDrawable()
        else:
            self.closeDrawable()
            
    def updateDrawable(self):
        
        if  self._status == self.OPENED:
            self.setFixedHeight(self.getDrawableHeight())
    
    def onAppendAlarm(self, alarm):
        
        widget = AlarmWidget(self, alarm)
        self.alarm_listview.addItem(widget.getItem())
        self.alarm_listview.setItemWidget(widget.getItem(), widget)
        
        if self.alarm_listview.count() > 1:
            self.updateHandleText(self.alarm_listview.count())
            self.drawable_button.setVisible(True)
        else:
            self.drawable_button.setVisible(False)
        
        self._parent.alarm_label.setVisible(True)
        self.setVisible(True)
        
        self.updateDrawable()
    
    def onRemoveAlarm(self, item):
        
        self.alarm_listview.takeItem(self.alarm_listview.row(item))
        
        if self.alarm_listview.count() == 1:
            self.drawable_button.setVisible(False)
            self.closeDrawable()
        elif self.alarm_listview.count() == 0:
            self._parent.alarm_label.setVisible(False)
            self.setVisible(False)
        else:
            self.updateHandleText(self.alarm_listview.count())
            self.updateDrawable()
            
    def updateHandleText(self, nb_alarm):
        self.drawable_button.setText("%s %s"%(nb_alarm,R.values.strings.alarms_waiting(self._lng)))
    
    def onTranslate(self, lng):
        self._lng = lng
        self.updateHandleText(self.alarm_listview.count())
        
    
#End of file
