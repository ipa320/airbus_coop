#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : plugin_installer.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

import rospy
import uuid
import os
from roslib.packages import get_pkg_dir
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from cobot_gui.account import Privilege, User
from cobot_gui.util import CobotGuiException, Parameters


from pyqt_agi_extend.QtAgiCore import get_pkg_dir_from_prefix
from pyqt_agi_extend.QtAgiGui import QAgiPopup

from cobot_gui.res import R


class DashboardPopup(QAgiPopup):
    
    def __init__(self,
                 parent,
                 popup_winpos=QAgiPopup.TopRight,
                 parent_winpos=QAgiPopup.BottomRight):
        
        QAgiPopup.__init__(self, parent)
        
        self.setRelativePosition(popup_winpos,
                                 parent_winpos)
        
        self._parent = parent
        
    def getParent(self):
        return self._parent
        
    def onCreate(self, param):
        pass
    
    def onTranslate(self, lng):
        pass
    
    def onDestroy(self):
        pass
    
    def closeEvent(self, event):
        self.onDestroy()

## @class WrapperDashboard
## @brief Base class for install base dashboard components.
class WrapperDashboard(QWidget):
    
    def __init__(self, context):
        QWidget.__init__(self)
        
        self._context = context
        self._name    = self.__class__.__name__
        
        self._param         = Parameters()
        self._popup_enabled = True
        self._access_rights = Privilege.OPERATOR
        
        self.setMinimumSize(QSize(35,35))
        self.setMaximumSize(QSize(600,35))
        
        self._layout = QHBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(5)
        
        context.addUserEventListener(self.onUserChanged)
        context.addLanguageEventListner(self.onTranslate)
        context.addControlModeEventListener(self.onControlModeChanged)
        context.addEmergencyStopEventListner(self.onEmergencyStop)
        context.addCloseEventListner(self.onDestroy)
        
    def setup(self, dashboard_descriptor, param):
        
        xsetup = dashboard_descriptor.find('setup')
        
        if xsetup is not None:
            
            icon = xsetup.find('icon')
            
            if icon is not None:
                icon_path = get_pkg_dir_from_prefix(icon.text)
                if os.path.isfile(icon_path):
                    ico_label = QLabel(self)
                    ico_label.setPixmap(QPixmap(icon_path).scaled(60,60))
            
            access_rights = xsetup.find('access-rights')
            
            if access_rights is not None:
                self._access_rights = Privilege.TOLEVEL[access_rights.text]
            
        else:
            self.logErr("Cannot found '<setup>' into %s/dashboard_descriptor.xml"%self.getName())
            
        self._param = param
        self.onCreate(param)
    
    def getContext(self):
        return self._context
    
    def getLayout(self):
        return self._layout
    
    def getName(self):
        return self._name
    
    def getAccessRights(self):
        return self._access_rights
    
    def setPopupEnabled(self, state):
        self._popup_enabled = state
    
    def logInfo(self,):
        self._context.getLogger().info(msg)
        
    def logWarn(self, msg):
        self._context.getLogger().warn(msg)
        
    def logErr(self, msg):
        self._context.getLogger().err(msg)
    
    def onCreate(self, param):
        raise NotImplementedError("Need to surchage onCreate(self, param)")
    
    def onControlModeChanged(self, mode):
        raise NotImplementedError("Need to surchage onControlModeChanged(self, mode)")
        
    def onUserChanged(self, user_info):
        raise NotImplementedError("Need to surchage onUserChanged(self, user_info)")
    
    def onTranslate(self, lng):
        raise NotImplementedError("Need to surchage onTranslate(self, lng)")
    
    def onEmergencyStop(self, state):
        raise NotImplementedError("Need to surchage onEmergencyStop(self, state)")
    
    def onDestroy(self):
        raise NotImplementedError("Need to surchage onDestroy(self)")
        
    def onRequestPopup(self):
        return None
    
    def mousePressEvent(self, event):
        if self._popup_enabled is True:
            popup = self.onRequestPopup()
            if popup is not None:
                popup.onCreate(self._param)
                popup.onTranslate(self.getContext().getLanguage())
                popup.adjustSize()
                popup.show_()
    
    def closeEvent(self, event):
        self.onDestroy()

#End of file

