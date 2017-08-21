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

DIR_TEMPLATEDASHBOARD_RESOURCES = os.path.join(get_pkg_dir('template_dashboard'),'resources')
DIR_TEMPLATEDASHBOARD_IMAGES = DIR_TEMPLATEDASHBOARD_RESOURCES+'/images'
DIR_TEMPLATEDASHBOARD_LAYOUTS = DIR_TEMPLATEDASHBOARD_RESOURCES+'/layouts'
DIR_TEMPLATEDASHBOARD_VALUES = DIR_TEMPLATEDASHBOARD_RESOURCES+'/values'

########################################
# Class(ies) declaration
########################################

class TemplateDashboardImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TEMPLATEDASHBOARD_IMAGES
        self.ic_dashboard = DIR_TEMPLATEDASHBOARD_IMAGES+'/ic_dashboard.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TemplateDashboardLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TEMPLATEDASHBOARD_LAYOUTS
        self.popup = DIR_TEMPLATEDASHBOARD_LAYOUTS+'/popup.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TemplateDashboardValues():
    def __init__(self):
        class TemplateDashboardStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def hello(self, lng="en"):
                if lng == "en":
                    return "Hello world! This is a template dashboard!".decode('utf-8')
                elif lng == "fr":
                    return "Bonjour le monde! Ceci est un template dashboard!".decode('utf-8')
                elif lng == "de":
                    return "Hallo Welt! Das ist ein Template Dashboard!".decode('utf-8')
                elif lng == "es":
                    return "Hola Mundo! Este es un dashboard de plantilla!".decode('utf-8')
                else:
                    return "Hello world! This is a template dashboard!".decode('utf-8')
                
        class TemplateDashboardStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.hello = "background-color:#d9d9d9;border-radius: 5px;font-size: 18pt; font-weight:40; color: #7c7c7c;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_TEMPLATEDASHBOARD_VALUES
        self.strings = TemplateDashboardStrings()
        self.styles = TemplateDashboardStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_TEMPLATEDASHBOARD_RESOURCES
    images = TemplateDashboardImages()
    layouts = TemplateDashboardLayouts()
    values = TemplateDashboardValues()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file
