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
    
