#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : scxml.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

import os
import rospy

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from pyqt_agi_extend import QtAgiCore

# Load my resources file
from res import R

class SCXMLState(QStandardItem):
    
    ROOT_STATE   = 0
    SIMPLE_STATE = 1
    PARENT_STATE = 2
    
    NOPE = ""
    RECUSIVE = '-r'
    
    PENDING = 0
    ACTIVED = 1
    ERROR   = -1
    
    def __init__(self, name, type=1):
        QStandardItem.__init__(self, name)
        self.setEditable(False)
        
        self._type   = type
        self._name   = name 
        self._status = self.PENDING
        
        self.setup()
        
    def setType(self, type):
        self._type = type
        self.refresh()
        
    def type(self):
        return self._type
    
    def setStatus(self, status, opt=""):
        
        self._status = status
        
        if self._type == self.SIMPLE_STATE:
            
            if status == 0:
                self.setIcon(R.getIconById("ic_unactive_state"))
            elif status == 1:
                self.setIcon(R.getIconById("ic_active_state"))
            elif status == -1:
                self.setIcon(R.getIconById("ic_pause_state"))
            else:
                self.setIcon(R.getIconById("ic_ssm_in_error"))
                
        elif self._type == self.PARENT_STATE:
            
            if status == 0:
                self.setIcon(R.getIconById("ic_unactive_parent_state"))
            elif status == 1:
                self.setIcon(R.getIconById("ic_active_parent_state"))
            elif status == -1:
                self.setIcon(R.getIconById("ic_pause_parent_state"))
            else:
                self.setIcon(R.getIconById("ic_ssm_in_error"))
    
    def status(self):
        return self._status
    
    def get_name(self):
        return self._name
    
    def refresh(self):
        self.setStatus(self.status())
        
    setup = refresh
    