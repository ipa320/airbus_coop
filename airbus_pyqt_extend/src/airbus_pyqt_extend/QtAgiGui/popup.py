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

from python_qt_binding import QtGui
from python_qt_binding import QtCore
from python_qt_binding.QtWidgets import *

## @package: QAgiPopup
##
## @version 2.0
## @author  Matignon Martin
## @date    Last modified 04/03/2014
## @class QAgiPopup
## @brief Popup object.
class QAgiPopup(QWidget):

    TopLeft     = 1
    BottomRight = 2
    TopRight    = 3
    BottomLeft  = 4
    Center      = 5
    
    def __init__(self, parent = None):
        """! The constructor.
        @param parent: object parent.
        @type parent: QObject.
        """
        
        QWidget.__init__(self, parent)
        self.setWindowFlags(QtCore.Qt.Popup | QtCore.Qt.FramelessWindowHint)
        self.setStyleSheet("QWidget{background-color: #d9d9d9;}")
        self._parent = parent
        self._popup_link  = self.TopRight
        self._parent_link = self.BottomRight
    
    @staticmethod
    def getObjectPosition(obj, corner):
        """! Get glabal corner position.
        @param obj: object.
        @type obj: QObject.
        
        @param corner: corner type.
        @type corner: int.
        """
        if corner == QAgiPopup.TopLeft:
            return obj.mapToGlobal(obj.rect().topLeft())
        elif corner == QAgiPopup.BottomRight:
            return obj.mapToGlobal(obj.rect().bottomRight())
        elif corner == QAgiPopup.TopRight:
            return obj.mapToGlobal(obj.rect().topRight())
        elif corner == QAgiPopup.BottomLeft:
            return obj.mapToGlobal(obj.rect().bottomLeft())
        else :
            return obj.mapToGlobal(obj.rect().center())
        
    def setRelativePosition(self, popup, widget):
        self._popup_link  = popup
        self._parent_link = widget
        
    def show_(self):
        
        """! Show popup."""
        parent_pos = QAgiPopup.getObjectPosition(self._parent, self._parent_link)
        popup_pos = QAgiPopup.getObjectPosition(self, self._popup_link)
        
        self.move(parent_pos - popup_pos)
        self.show()
        
#End of file
