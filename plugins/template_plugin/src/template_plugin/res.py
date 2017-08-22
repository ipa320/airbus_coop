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

DIR_TemplatePLUGIN_RESOURCES = os.path.join(get_pkg_dir('template_plugin'),'resources')
DIR_TemplatePLUGIN_IMAGES = DIR_TemplatePLUGIN_RESOURCES+'/images'
DIR_TemplatePLUGIN_VALUES = DIR_TemplatePLUGIN_RESOURCES+'/values'
DIR_TemplatePLUGIN_LAYOUTS = DIR_TemplatePLUGIN_RESOURCES+'/layouts'

########################################
# Class(ies) declaration
########################################

class TemplatePluginImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TemplatePLUGIN_IMAGES
        self.ic_launcher = DIR_TemplatePLUGIN_IMAGES+'/ic_launcher.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TemplatePluginValues():
    def __init__(self):
        class TemplatePluginStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.hello = "background-color:#d9d9d9;border-radius: 5px;font-size: 18pt; font-weight:40; color: #7c7c7c;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        class TemplatePluginStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def hello(self, lng="en"):
                if lng == "en":
                    return "Hello world! This is a template plugin!".decode('utf-8')
                elif lng == "fr":
                    return "Bonjour le monde! Ceci est un template plugin!".decode('utf-8')
                elif lng == "de":
                    return "Hallo Welt! Das ist ein Template Plugin!".decode('utf-8')
                elif lng == "es":
                    return "Hola Mundo! Este es un plugin de plantilla!".decode('utf-8')
                else:
                    return "Hello world! This is a template plugin!".decode('utf-8')
                
        self.uuid = self.__class__.__name__
        self.dir = DIR_TemplatePLUGIN_VALUES
        self.styles = TemplatePluginStyles()
        self.strings = TemplatePluginStrings()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TemplatePluginLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TemplatePLUGIN_LAYOUTS
        self.mainwindow = DIR_TemplatePLUGIN_LAYOUTS+'/mainwindow.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_TemplatePLUGIN_RESOURCES
    images = TemplatePluginImages()
    values = TemplatePluginValues()
    layouts = TemplatePluginLayouts()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file
