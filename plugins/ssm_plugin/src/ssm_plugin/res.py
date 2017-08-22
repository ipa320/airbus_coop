#!/usr/bin/env python
# -*- coding: utf-8 -*-
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


########################################
# Module(s) declaration
########################################

import rospy
import os
from roslib.packages import get_pkg_dir
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

########################################
# Constante(s) and Variable(s) declaration
########################################

DIR_SSMPLUGIN_RESOURCES = os.path.join(get_pkg_dir('ssm_plugin'),'resources')
DIR_SSMPLUGIN_VALUES = DIR_SSMPLUGIN_RESOURCES+'/values'
DIR_SSMPLUGIN_IMAGES = DIR_SSMPLUGIN_RESOURCES+'/images'
DIR_SSMPLUGIN_LAYOUTS = DIR_SSMPLUGIN_RESOURCES+'/layouts'

########################################
# Class(ies) declaration
########################################

class SsmPluginValues():
    def __init__(self):
        class SsmPluginStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def title(self, lng="en"):
                if lng == "en":
                    return "Smart State Machine Plugin interface".decode('utf-8')
                elif lng == "fr":
                    return "Interface pour Machine à états intelligente".decode('utf-8')
                else:
                    return "Smart State Machine Plugin interface".decode('utf-8')
                
            def scxml_header(self, lng="en"):
                if lng == "en":
                    return "SCXML".decode('utf-8')
                elif lng == "fr":
                    return "SCXML".decode('utf-8')
                else:
                    return "SCXML".decode('utf-8')
                
        class SsmPluginStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.label = "background-color:#d9d9d9;border-radius: 5px;font-size: 14pt;color: #7c7c7c;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_SSMPLUGIN_VALUES
        self.strings = SsmPluginStrings()
        self.styles = SsmPluginStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class SsmPluginImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_SSMPLUGIN_IMAGES
        self.ic_ssm_pause = DIR_SSMPLUGIN_IMAGES+'/ic_ssm_pause.png'
        self.ic_database = DIR_SSMPLUGIN_IMAGES+'/ic_database.png'
        self.ic_launcher = DIR_SSMPLUGIN_IMAGES+'/ic_launcher.png'
        self.ic_ssm_error = DIR_SSMPLUGIN_IMAGES+'/ic_ssm_error.png'
        self.ic_ssm_ok = DIR_SSMPLUGIN_IMAGES+'/ic_ssm_ok.png'
        self.ic_sub_ssm = DIR_SSMPLUGIN_IMAGES+'/ic_sub_ssm.png'
        self.ic_unactive_parent_state = DIR_SSMPLUGIN_IMAGES+'/ic_unactive_parent_state.png'
        self.ic_ssm_pending = DIR_SSMPLUGIN_IMAGES+'/ic_ssm_pending.png'
        self.ic_pause_parent_state = DIR_SSMPLUGIN_IMAGES+'/ic_pause_parent_state.png'
        self.ic_ready = DIR_SSMPLUGIN_IMAGES+'/ic_ready.gif'
        self.ic_start = DIR_SSMPLUGIN_IMAGES+'/ic_start.png'
        self.ic_pause_state = DIR_SSMPLUGIN_IMAGES+'/ic_pause_state.png'
        self.ic_state = DIR_SSMPLUGIN_IMAGES+'/ic_state.png'
        self.ic_pause = DIR_SSMPLUGIN_IMAGES+'/ic_pause.png'
        self.ic_stop = DIR_SSMPLUGIN_IMAGES+'/ic_stop.png'
        self.ic_folder = DIR_SSMPLUGIN_IMAGES+'/ic_folder.png'
        self.ic_rearm = DIR_SSMPLUGIN_IMAGES+'/ic_rearm.png'
        self.ic_active_state = DIR_SSMPLUGIN_IMAGES+'/ic_active_state.png'
        self.ic_active_parent_state = DIR_SSMPLUGIN_IMAGES+'/ic_active_parent_state.png'
        self.ic_ssm = DIR_SSMPLUGIN_IMAGES+'/ic_ssm.png'
        self.ic_unactive_state = DIR_SSMPLUGIN_IMAGES+'/ic_unactive_state.png'
        self.ic_preempt = DIR_SSMPLUGIN_IMAGES+'/ic_preempt.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class SsmPluginLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_SSMPLUGIN_LAYOUTS
        self.mainwindow = DIR_SSMPLUGIN_LAYOUTS+'/mainwindow.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_SSMPLUGIN_RESOURCES
    values = SsmPluginValues()
    images = SsmPluginImages()
    layouts = SsmPluginLayouts()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file
