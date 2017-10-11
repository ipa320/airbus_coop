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
from roslib.packages import get_pkg_dir
import math

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *

from airbus_pyqt_extend.QtAgiCore import loadRsc

rsc = loadRsc("airbus_pyqt_extend")

class QAgiJoystickHandle(QLabel):
    
    def __init__(self, parent=None, w=100, h=100):
        QLabel.__init__(self, parent)
        
        self.setWindowFlags(Qt.Window | Qt.FramelessWindowHint)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)
        self.setAttribute(Qt.WA_NoSystemBackground)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setFixedSize(QSize(w, h))
        
        self.setPixmap(rsc.images.getPixmap("jHand",w, h))
        
        self._x0 = w / 2
        self._y0 = h / 2
        
    def follow(self, x, y):
        self.move(x-self._x0, y-self._y0)
        
    def stop(self, x0, y0):
        self.move(x0-self._x0, y0-self._y0)
        
    def x0(self):
        return self._x0
    
    def y0(self):
        return self._y0
    
    def resizeEvent(self, event):
        self._x0 = self.width() / 2
        self._y0 = self.height() / 2
        self.setPixmap(rsc.images.getPixmap("jHand",self.width(), self.height()))
    
class QAgiAbstractJoystick(QLabel):
    
    def __init__(self, parent=None, listener=None):
        QLabel.__init__(self, parent)
        
        self.setWindowFlags(Qt.Window | Qt.FramelessWindowHint)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)
        self.setAttribute(Qt.WA_NoSystemBackground)
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setAttribute(Qt.WA_AcceptTouchEvents)
        self.setFixedSize(QSize(256,256))
        
        self._listener = listener
        
        self._handle = QAgiJoystickHandle(self, 90, 90)
        
        self._rayon = self.width()/ 4
        self._x0    = self.width() / 2
        self._y0    = self.height() / 2
        
        self._background = None
        
        self._handle.stop(self._x0, self._y0)
        
    def setBackgroundRsc(self, name):
        self._background = name
        self.setPixmap(rsc.images.getPixmap( self._background, self.width(), self.height()))
        
    def setHandleSize(self, size):
        self._handle.setFixedSize(size)
        self._handle.resizeEvent(None)
        self._handle.stop(self._x0, self._y0)
        
    def setHandleEventListener(self, listener):
        self._listener = listener
        
    def mousePressEvent(self, event):
        self.emit(SIGNAL('joystickPressedEvent'))
        
    def mouseMoveEvent(self, event):
        
        try:
            if self._listener is not None:
                self._listener(event)
        except Exception as ex:
            rospy.logerr(str(ex))
        
    def mouseReleaseEvent(self, event):
        self._handle.stop(self._x0, self._y0)
        self.emit(SIGNAL('valuesChanged'),[0, 0])
        self.emit(SIGNAL('joystickReleasedEvent'))
        
    def x0(self):
        return self._x0
    
    def y0(self):
        return self._y0
    
    def getRadius(self):
        return self._rayon
    
    def getHandle(self):
        return self._handle
    
    def resizeEvent(self, event):
        self._rayon = self.width()/ 4
        self._x0    = self.width() / 2
        self._y0    = self.height() / 2
        self._handle.stop(self._x0, self._y0)
        self.setPixmap(rsc.images.getPixmap( self._background, self.width(), self.height()))
    

class QAgiJoystick(QAgiAbstractJoystick):
    
    def __init__(self, parent=None):
        QAgiAbstractJoystick.__init__(self, parent)
        self.setBackgroundRsc("joyXY")
        self.setHandleEventListener(self._handle_event_listner)
        
    
    def _handle_event_listner(self, event):
        
        x = event.pos().x()
        y = event.pos().y()
        B = event.pos().x() - self.x0()
        A = self._y0 - event.pos().y()
        
        th = math.sqrt((A*A) + (B*B))
        
        if th < self.getRadius():
            x = event.pos().x()
            y = event.pos().y()
        else:
            
            try:
                AE = (self.getRadius() / th)*B
                DE = (AE / B)*A
                x = AE+self.x0()
                y = -DE+self.y0()
            except ZeroDivisionError:
                pass
            
        self.getHandle().follow(x, y)
        
        x = float((x - self.getHandle().x0())-(self.x0()- self.getHandle().x0())) / self.getRadius()
        y = float((self.y0()- self.getHandle().y0())-(y - self.getHandle().y0())) / self.getRadius()
        
        self.emit(SIGNAL('valuesChanged'),[x, y])

class QAgiHJoystick(QAgiAbstractJoystick):
    
    def __init__(self, parent=None):
        QAgiAbstractJoystick.__init__(self, parent)
        self.setBackgroundRsc("joyX")
        self.setHandleEventListener(self._handle_event_listner)
    
    def _handle_event_listner(self, event):
        
        x = event.pos().x()
        
        if event.pos().x() > self.x0()-self.getRadius() and \
           event.pos().x() < self.x0()+self.getRadius():
            
            self.getHandle().follow(x, self.y0())
            
            x = float((x - self.getHandle().x0())-(self.x0()- self.getHandle().x0())) / self.getRadius()
            self.emit(SIGNAL('valuesChanged'),[x, 0.])

class QAgiVJoystick(QAgiAbstractJoystick):
    
    def __init__(self, parent=None):
        QAgiAbstractJoystick.__init__(self, parent)
        self.setBackgroundRsc("joyY")
        self.setHandleEventListener(self._handle_event_listner)
    
    def _handle_event_listner(self, event):
        
        y = event.pos().y()
        
        if y > self.y0()-self.getRadius() and \
           y < self.y0()+self.getRadius():
            
            self.getHandle().follow(self.x0(), y)
            
            y = float((self.y0()- self.getHandle().y0())-(y - self.getHandle().y0())) / self.getRadius()
            self.emit(SIGNAL('valuesChanged'),[0., y])
            
    def resizeEvent(self, event):
        self.setPixmap(rsc.images.getPixmap("joyY", self.width(), self.height()))
            
class Unitest(QWidget):
    
    def __init__(self):
        QWidget.__init__(self)
        
        layout = QHBoxLayout(self)
        
        m_joystick   = QAgiJoystick()
        m_joystick.setHandleSize(QSize(150,150))
        m_joystick.setFixedSize(QSize(512,512))
        
        m_h_joystick = QAgiHJoystick()
        m_v_joystick = QAgiVJoystick()
        
        self.connect(m_joystick, SIGNAL("valuesChanged"),   self._update_values)
        self.connect(m_h_joystick, SIGNAL("valuesChanged"), self._update_values)
        self.connect(m_v_joystick, SIGNAL("valuesChanged"), self._update_values)
        
        layout.addWidget(m_joystick)
        layout.addWidget(m_h_joystick)
        layout.addWidget(m_v_joystick)
        
    def _update_values(self, values):
        print str(values)

if __name__ == "__main__":
    
    import sys, os
    
    a = QApplication(sys.argv)
    
    name = 'omni_gui_test_node_%d' % os.getpid()
    rospy.init_node(name)
    MainWindow = Unitest()
    MainWindow.show()
    a.exec_()
#End of file

