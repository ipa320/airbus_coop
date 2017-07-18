#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : keyboard.py
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
import sys
from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiCore import loadRsc

rsc = loadRsc("pyqt_agi_extend")

class NumericFloatKeyboard(QDialog):
    
    def __init__(self, parent):
        QDialog.__init__(self, parent, Qt.FramelessWindowHint)
        
        self._parent = parent
        
        # Extend the widget with all attributes and children from UI file
        rsc.uis.load("keyboard", self)
        
        self.adjustSize()
        
        self.connect(self.nb1, SIGNAL("clicked()"), self.number_1)
        self.connect(self.nb2, SIGNAL("clicked()"), self.number_2)
        self.connect(self.nb3, SIGNAL("clicked()"), self.number_3)
        self.connect(self.nb4, SIGNAL("clicked()"), self.number_4)
        self.connect(self.nb5, SIGNAL("clicked()"), self.number_5)
        self.connect(self.nb6, SIGNAL("clicked()"), self.number_6)
        self.connect(self.nb7, SIGNAL("clicked()"), self.number_7)
        self.connect(self.nb8, SIGNAL("clicked()"), self.number_8)
        self.connect(self.nb9, SIGNAL("clicked()"), self.number_9)
        self.connect(self.nb0, SIGNAL("clicked()"), self.number_0)
        self.connect(self.point, SIGNAL("clicked()"), self.point_float)
        self.connect(self.clear, SIGNAL("clicked()"), self._clear)
        self.connect(self.clear, SIGNAL("toggled()"), self._clear_all)
        
        self.connect(self.ok, SIGNAL("clicked()"), self._ok)
        
        self.num_res.setText(self._parent.text())
        
    def number_1(self):
        self.num_res.setText(self.num_res.text()+'1')
        
    def number_2(self):
        self.num_res.setText(self.num_res.text()+'2')
        
    def number_3(self):
        self.num_res.setText(self.num_res.text()+'3')
        
    def number_4(self):
        self.num_res.setText(self.num_res.text()+'4')
        
    def number_5(self):
        self.num_res.setText(self.num_res.text()+'5')
        
    def number_6(self):
        self.num_res.setText(self.num_res.text()+'6')
        
    def number_7(self):
        self.num_res.setText(self.num_res.text()+'7')
        
    def number_8(self):
        self.num_res.setText(self.num_res.text()+'8')
        
    def number_9(self):
        self.num_res.setText(self.num_res.text()+'9')
        
    def number_0(self):
        self.num_res.setText(self.num_res.text()+'0')
        
    def point_float(self):
        self.num_res.setText(self.num_res.text()+'.')
        
    def _clear(self):
        self.num_res.setText(self.num_res.text()[:-1])
        
    def _clear_all(self):
        self.num_res.setText('')
        
    def _ok(self):
        self._parent.setText(self.num_res.text())
        self.close()
        
class QAgiKeyboard:
    
    NumericInt   = 1
    NumericFloat = 2
    Alphanumeric = 3
    
    def __init__(self, parent, flag):
        
        if flag == self.NumericInt:
            pass
        elif flag == self.NumericFloat:
            keyboard = NumericFloatKeyboard(parent)
            keyboard.show()
        elif flag == self.Alphanumeric:
            pass
        
##@cond
##Unit test
if __name__ == "__main__":
    
    import sys
    
    class Widget(QLineEdit):
        def __init__(self):
            QLineEdit.__init__(self)
        
        def mousePressEvent(self, event):
            QAgiKeyboard(self, QAgiKeyboard.NumericFloat)
    
    a = QApplication(sys.argv)
    utt_appli = QMainWindow()
    
    account_ui = Widget()
    
    utt_appli.setCentralWidget(Widget())
    
    utt_appli.show()
    a.exec_()
#@endcond

#End of file