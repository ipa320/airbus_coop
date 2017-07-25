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

DIR_UR10EFFECTORPLUGIN_RESOURCES = os.path.join(get_pkg_dir('ur10_effector_plugin'),'resources')
DIR_UR10EFFECTORPLUGIN_IMAGES = DIR_UR10EFFECTORPLUGIN_RESOURCES+'/images'
DIR_UR10EFFECTORPLUGIN_LAYOUTS = DIR_UR10EFFECTORPLUGIN_RESOURCES+'/layouts'
DIR_UR10EFFECTORPLUGIN_VALUES = DIR_UR10EFFECTORPLUGIN_RESOURCES+'/values'

########################################
# Class(ies) declaration
########################################

class Ur10EffectorPluginImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_UR10EFFECTORPLUGIN_IMAGES
        self.ic_launcher = DIR_UR10EFFECTORPLUGIN_IMAGES+'/ic_launcher.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class Ur10EffectorPluginLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_UR10EFFECTORPLUGIN_LAYOUTS
        self.mainwindow = DIR_UR10EFFECTORPLUGIN_LAYOUTS+'/mainwindow.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class Ur10EffectorPluginValues():
    def __init__(self):
        class Ur10EffectorPluginStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def hello(self, lng="en"):
                if lng == "en":
                    return "Hello world !".decode('utf-8')
                elif lng == "fr":
                    return "Bonjour le monde !".decode('utf-8')
                elif lng == "de":
                    return "Hallo Welt !".decode('utf-8')
                elif lng == "es":
                    return "Hola Mundo !".decode('utf-8')
                else:
                    return "Hello world !".decode('utf-8')
                
        class Ur10EffectorPluginStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.hello = "background-color:#d9d9d9;border-radius: 5px;font-size: 18pt; font-weight:40; color: #7c7c7c;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_UR10EFFECTORPLUGIN_VALUES
        self.strings = Ur10EffectorPluginStrings()
        self.styles = Ur10EffectorPluginStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_UR10EFFECTORPLUGIN_RESOURCES
    images = Ur10EffectorPluginImages()
    layouts = Ur10EffectorPluginLayouts()
    values = Ur10EffectorPluginValues()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file