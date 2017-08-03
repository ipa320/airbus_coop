#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : QAgiPopup.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import rospy

from python_qt_binding import QtGui
from python_qt_binding import QtCore

## @package: QAgiPopup
##
## @version 2.0
## @author  Matignon Martin
## @date    Last modified 04/03/2014
## @class QAgiPopup
## @brief Popup object.
class QAgiPopup(QtGui.QWidget):

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
        
        QtGui.QWidget.__init__(self, parent)
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
