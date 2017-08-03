#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : res.py
# Authors : Martin Matignon
#
################################################################################


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

DIR_OMNIGUI_RESOURCES = os.path.join(get_pkg_dir('omni_gui'),'resources')
DIR_OMNIGUI_IMAGES = DIR_OMNIGUI_RESOURCES+'/images'
DIR_OMNIGUI_LAYOUTS = DIR_OMNIGUI_RESOURCES+'/layouts'
DIR_OMNIGUI_VALUES = DIR_OMNIGUI_RESOURCES+'/values'
DIR_OMNIGUI_YAML = DIR_OMNIGUI_RESOURCES+'/yaml'

########################################
# Class(ies) declaration
########################################

class OmniGuiImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_OMNIGUI_IMAGES
        self.localization = DIR_OMNIGUI_IMAGES+'/localization.png'
        self.rotation = DIR_OMNIGUI_IMAGES+'/rotation.png'
        self.icon_omni = DIR_OMNIGUI_IMAGES+'/icon_omni.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class OmniGuiLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_OMNIGUI_LAYOUTS
        self.navigation_dashbord = DIR_OMNIGUI_LAYOUTS+'/navigation_dashbord.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class OmniGuiValues():
    def __init__(self):
        class OmniGuiStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def title(self, lng="en"):
                if lng == "en":
                    return "Omni".decode('utf-8')
                elif lng == "fr":
                    return "Omni".decode('utf-8')
                else:
                    return "Omni".decode('utf-8')
                
        class OmniGuiStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.rviz = "background-color: #1c2434;border-radius: 5px;"
                self.dashboard_background = "background-color:qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #1c2434, stop: 1 #4d596f);border-radius:5px;"
                self.dashboard_text = "background-color: transparent;font-size: 18pt;font-weight:40;color: #ffffff;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_OMNIGUI_VALUES
        self.strings = OmniGuiStrings()
        self.styles = OmniGuiStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class OmniGuiYaml():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_OMNIGUI_YAML
        self.default = DIR_OMNIGUI_YAML+'/default.rviz'
        self.map = DIR_OMNIGUI_YAML+'/map.rviz'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_OMNIGUI_RESOURCES
    images = OmniGuiImages()
    layouts = OmniGuiLayouts()
    values = OmniGuiValues()
    yaml = OmniGuiYaml()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file