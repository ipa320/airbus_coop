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

DIR_TUTOPLUGIN_RESOURCES = os.path.join(get_pkg_dir('tuto_plugin'),'resources')
DIR_TUTOPLUGIN_IMAGES = DIR_TUTOPLUGIN_RESOURCES+'/images'
DIR_TUTOPLUGIN_TEST = DIR_TUTOPLUGIN_RESOURCES+'/test'
DIR_TUTOPLUGIN_LAYOUTS = DIR_TUTOPLUGIN_RESOURCES+'/layouts'
DIR_TUTOPLUGIN_VALUES = DIR_TUTOPLUGIN_RESOURCES+'/values'

########################################
# Class(ies) declaration
########################################

class TutoPluginImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TUTOPLUGIN_IMAGES
        self.pere_noel2 = DIR_TUTOPLUGIN_IMAGES+'/pere_noel2.gif'
        self.ic_launcher = DIR_TUTOPLUGIN_IMAGES+'/ic_launcher.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TutoPluginTest():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TUTOPLUGIN_TEST
        self.test1 = DIR_TUTOPLUGIN_TEST+'/test1.txt'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TutoPluginLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TUTOPLUGIN_LAYOUTS
        self.mainwindow = DIR_TUTOPLUGIN_LAYOUTS+'/mainwindow.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TutoPluginValues():
    def __init__(self):
        class TutoPluginStrings():
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
                
            def tuto(self, lng="en"):
                if lng == "en":
                    return "Hello tuto !".decode('utf-8')
                elif lng == "fr":
                    return "Bonjour tuto !".decode('utf-8')
                else:
                    return "Hello tuto !".decode('utf-8')
                
        class TutoPluginStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.hello = "background-color:#d9d9d9;border-radius: 5px;font-size: 18pt; font-weight:40; color: #7c7c7c;"
                self.tuto = "background-color:rgb(255,0,0);border-radius: 5px;font-size: 32pt; font-weight:40; color: #7c7c7c;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_TUTOPLUGIN_VALUES
        self.strings = TutoPluginStrings()
        self.styles = TutoPluginStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_TUTOPLUGIN_RESOURCES
    images = TutoPluginImages()
    test = TutoPluginTest()
    layouts = TutoPluginLayouts()
    values = TutoPluginValues()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file