#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : qsilderbutton.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

import rospy

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from pyqt_agi_extend.QtAgiCore import loadRsc

rsc = loadRsc("pyqt_agi_extend")

## @package: qsilderbutton
##
## @version 2.0
## @author  Matignon Martin
## @date    Last modified 04/03/2014
## @class QPopup
## @brief Slider button object.
class QAgiSilderButton(QLabel):
    
    TRIGGER = 'statusChanged'
    
    BACKGROUND_CSS = rsc.styles["QAgiSilderButton.background"]
    ON_CSS         = rsc.styles["QAgiSilderButton.on"]
    OFF_CSS        = rsc.styles["QAgiSilderButton.off"]
     
    def __init__(self,parent=None, initial_state=False, on_label='ON', off_label='OFF'):
        """! The constructor.
        @param parent: object parent.
        @type parent: QObject.
         
        @param initial_state: button state (False = off/ True = on).
        @type initial_state: bool.
        """
         
        QLabel.__init__(self, parent)
        
        self._on_label  = on_label
        self._off_label = off_label
        self._th_off    = None
        self._th_on     = None
        self._on        = None
        self._off       = None
        self._stick_off = None
        self._stick_on  = None
        self._pressed   = False
        
        self.setStyleSheet(self.BACKGROUND_CSS)
         
        self._trigger = QLabel(self)
        self._trigger.setMouseTracking(True)
          
        self._trigger.resize(self.width()/2+1,self.height())
        self._th_off  = self._trigger.width()/2
        self._th_on   = self._th_off+1
        self._on      = self.width()-self._trigger.width()
        self._off     = 0
        self._stick_off = int(0.3*self._on)
        self._stick_on  = int(2*self._stick_off)
          
        if initial_state == True:
            self._trigger.setStyleSheet(self.ON_CSS)# %(self.height()/4))
            self._trigger.setText(self._on_label)
            self._trigger.move(self._on,self._trigger.y())
        else:
            self._trigger.setStyleSheet(self.OFF_CSS)# %(self.height()/4))
            self._trigger.setText(self._off_label)
            
    def setBackgroundStyle(self, style):
        self.BACKGROUND_CSS = style
        
    def setOnText(self, text):
        self._on_label = text
        self.setState(self.getState(),True)
        
    def setOnStyle(self, style):
        self.ON_CSS = style
        
    def setOffText(self, text):
        self._off_label = text
        self.setState(self.getState(),True)
        
    def setOffStyle(self, style):
        self.OFF_CSS = style
    
    def getState(self):
        """! Get button state.
        @param state: button state (True/False).
        @type state: bool.
        """
        if self._trigger.x() < self._th_off:
            return False
        else:
            return True
         
    def setState(self, state , anonymous=False):
        """! Set button state.
        @return state: button state (False = off/ True = on).
        @type state: bool.
        
        @return anonymous: Option to enabled/desibled QT SIGNAL('statusChanged').
        @type anonymous: bool.
        """
        if state:
            self._trigger.move(self._on,self._trigger.y())
            self._trigger.setStyleSheet(self.ON_CSS)# %(self.height()/4))
            self._trigger.setText(self._on_label)
            
            if anonymous is False:
                self.emit(SIGNAL('statusChanged'), True)
        else:
            self._trigger.move(self._off,self._trigger.y())
            self._trigger.setStyleSheet(self.OFF_CSS)# %(self.height()/4))
            self._trigger.setText(self._off_label)
            
            if anonymous is False:
                self.emit(SIGNAL('statusChanged'), False)
     
    def mousePressEvent(self, event):
        """! Detect button pressed.
        @param event: event.
        @type event: QEvent.
        """
        self._pressed = True
         
    def mouseReleaseEvent(self, event):
        """! Detect button released.
        @param event: event.
        @type event: QEvent
        """
        self._pressed = False
        
        x = event.pos().x() - self._trigger.width()/2
        
        if x >= self._th_on:
            self._trigger.move(self._on,self._trigger.y())
            self._trigger.setStyleSheet(self.ON_CSS)# %(self.height()/4))
            self._trigger.setText(self._on_label)
            self.emit(SIGNAL('statusChanged'),True)
        elif x <= self._th_off:
            self._trigger.move(self._off,self._trigger.y())
            self._trigger.setStyleSheet(self.OFF_CSS)# %(self.height()/4))
            self._trigger.setText(self._off_label)
            self.emit(SIGNAL('statusChanged'),False)
         
    def mouseMoveEvent(self, event):
        """! Detect button moved.
        @param event: event.
        @type event: QEvent
        """
        
        x = event.pos().x() - self._trigger.width()/2
        if self._pressed and x >= self._off and x <= self._on:
            if x > self._stick_on:
                self._trigger.move(self._on,self._trigger.y())
                self._trigger.setStyleSheet(self.ON_CSS)# %(self.height()/4))
                self._trigger.setText(self._on_label)
            elif x < self._stick_off:
                self._trigger.move(self._off,self._trigger.y())
                self._trigger.setStyleSheet(self.OFF_CSS)# %(self.height()/4))
                self._trigger.setText(self._off_label)
            else:
                self._trigger.move(x,self._trigger.y())
                 
    def resizeEvent(self,event):
        """! Resize button.
        @param event: event.
        @type event: QEvent
        """
         
        if self.width() % 2 == 0:
            self.resize(self.width()+1,self.height())
        else:
            self.resize(self.width(),self.height())
             
        if self.getState():
            self._trigger.setStyleSheet(self.ON_CSS)# %(self.height()/4))
        else:
            self._trigger.setStyleSheet(self.OFF_CSS)# %(self.height()/4))
             
        self._trigger.resize(self.width()/2+1,self.height())
        self._th_off  = self._trigger.width()/2
        self._th_on   = self._th_off+1
        self._on      = self.width()-self._trigger.width()
        self._off     = 0
        self._stick_off = int(0.3*self._on)
        self._stick_on  = int(2*self._stick_off)
         
        if self._trigger.x() != 0:
            self._trigger.move(self._on,self._trigger.y())
         
if __name__ == "__main__":
    
    from python_qt_binding.QtGui import QApplication
    import sys
    
    rospy.init_node('utt_agi_silderbutton')
    
    app = QApplication(sys.argv)
    MainWindow = QAgiSilderButton()
    
    def trigger(state):
        print state
    
    QObject.connect(MainWindow, SIGNAL('statusChanged'), trigger)
    
    MainWindow.show()
    app.exec_()
    
#End of file
