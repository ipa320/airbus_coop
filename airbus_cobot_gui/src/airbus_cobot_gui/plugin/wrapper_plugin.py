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
import uuid
import os
from roslib.packages import get_pkg_dir
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *

from airbus_cobot_gui.account import Privilege, User
from airbus_cobot_gui.alarm import Alarm
from airbus_cobot_gui.util import CobotGuiException

from airbus_pyqt_extend.QtAgiCore import get_pkg_dir_from_prefix

from airbus_cobot_gui.res import R

## @package: plugin
##
## @version 4.0
## @author  Matignon Martin
## @date    Last modified 28/02/2014

class LauncherPlugin(QPushButton):
    
    def __init__(self, plugin):
        QPushButton.__init__(self)
        
        self.setFocusPolicy(Qt.NoFocus)
        self.setObjectName(plugin.getPluginName())
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.setEnabled(False)
        
        self._plugin = plugin
        self._access_rights = Privilege.OPERATOR
        
        plugin.getContext().addUserEventListener(self.onUserChanged)
        
    def setup(self, xsetup):
        
        xicon = xsetup.find('icon')
         
        if xicon is not None:
             
            ic_path = get_pkg_dir_from_prefix(xicon.text)
             
            if os.path.isfile(ic_path):
                self.setIcon(QIcon(ic_path))
                self.setIconSize(QSize(80,80))
            else:
                self.setStyleSheet(R.values.styles.default_launch)
                self.setText(self._plugin.getPluginName())
        else:
            self.logErr("Cannot found '<icon>' into %s/plugin_descriptor.xml"%self.objectName())
            
        xaccess = xsetup.find('access-rights')
        
        if xaccess is not None:
            try:
                self._access_rights = Privilege.TOLEVEL[xaccess.text.lower()]
            except:
                self._access_rights = Privilege.OPERATOR
                self.logWarn("Invalid access rights from %s"%self.objectName())
        else:
            self.logErr("Cannot found node '<access-rights>' into %s/plugin_descriptor.xml"%self.objectName())
        
    def onUserChanged(self, user):
        
        if user.getUserPrivilege() < self.getAccessRights():
            self.setEnabled(False)
        else:
            self.setEnabled(True)
    
    def setAccessRights(self, access_rights):
        self._access_rights = access_rights
        
    def getAccessRights(self):
        return self._access_rights
    
    def getPluginAttached(self):
        return self._plugin
    
    # Remap method name
    getView = getPluginAttached

## @class WrapperPlugin
## @brief Base class for install base plugin components.
class WrapperPlugin(QWidget):
    
    def __init__(self, context):
        QWidget.__init__(self)
        
        self._context = context
        self._plugin_name = str(self.__class__.__name__)
        self._launcher = LauncherPlugin(self)
        
        self.connect(self._launcher, SIGNAL('clicked()'), self.onRequestDisplayView)
        
        context.addUserEventListener(self.onUserChanged)
        context.addLanguageEventListner(self.onTranslate)
        context.addControlModeEventListener(self.onControlModeChanged)
        context.addEmergencyStopEventListner(self.onEmergencyStop)
        context.addCloseEventListner(self.tryToDestroy)
        
    def setup(self, plugin_descriptor, param):
        
        xsetup = plugin_descriptor.find('setup')
        
        if xsetup is not None:
            self._launcher.setup(xsetup)
        else:
            self.logErr("Cannot found '<setup>' into %s/plugin_descriptor.xml"%self._plugin_name)
            
        self.tryToCreate(param)
    
    def getContext(self):
        return self._context
    
    def getLauncher(self):
        return self._launcher
    
    def getPluginName(self):
        return self._plugin_name
    
    getName = getPluginName
    
    def logInfo(self,):
        self._context.getLogger().info(msg)
        
    def logWarn(self, msg):
        self._context.getLogger().warn(msg)
        
    def logErr(self, msg):
        self._context.getLogger().err(msg)
        
    def sendAlarm(self, level, msg):
        self._context.sendAlarm(level, msg)
        
    def onRequestDisplayView(self):
        self._context.requestDisplayView(self)
    
    def onCreate(self, param):
        pass
    
    def tryToCreate(self, param):
        
        try:
            self.onCreate(param)
        except Exception as ex:
            rospy.logerr("GUI ERROR : %s"%str(ex))
            self.getContext().sendAlarm(Alarm.WARNING, str(ex))
    
    def onPause(self):
        pass
    
    def tryToPause(self):
        
        try:
            self.onPause()
        except Exception as ex:
            
            rospy.logerr("GUI ERROR : %s"%str(ex))
            self.getContext().sendAlarm(Alarm.WARNING, str(ex))
    
    def onResume(self):
        pass
    
    def tryToResume(self):
        try:
            self.onResume()
        except Exception as ex:
            rospy.logerr("GUI ERROR : %s"%str(ex))
            self.getContext().sendAlarm(Alarm.WARNING, str(ex))
    
    def onControlModeChanged(self, mode):
        pass
        
    def onUserChanged(self, user_info):
        pass
    
    def onTranslate(self, lng):
        pass
    
    def onEmergencyStop(self, state):
        pass
    
    def onDestroy(self):
        raise NotImplementedError("Need to surchage onDestroy(self)")
    
    def tryToDestroy(self):
        rospy.logdebug("Closing : %s"%self._plugin_name)
        try:
            self.onDestroy()
        except Exception as ex:
            rospy.logerr("GUI ERROR : %s"%str(ex))
            self.getContext().sendAlarm(Alarm.WARNING, str(ex))
    
    def closeEvent(self, event):
        self.tryToDestroy()

#End of file

