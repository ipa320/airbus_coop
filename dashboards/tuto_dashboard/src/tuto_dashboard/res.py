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

DIR_TUTODASHBOARD_RESOURCES = os.path.join(get_pkg_dir('tuto_dashboard'),'resources')
DIR_TUTODASHBOARD_IMAGES = DIR_TUTODASHBOARD_RESOURCES+'/images'
DIR_TUTODASHBOARD_LAYOUTS = DIR_TUTODASHBOARD_RESOURCES+'/layouts'
DIR_TUTODASHBOARD_VALUES = DIR_TUTODASHBOARD_RESOURCES+'/values'

########################################
# Class(ies) declaration
########################################

class TutoDashboardImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TUTODASHBOARD_IMAGES
        self.ic_dashboard = DIR_TUTODASHBOARD_IMAGES+'/ic_dashboard.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TutoDashboardLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TUTODASHBOARD_LAYOUTS
        self.popup = DIR_TUTODASHBOARD_LAYOUTS+'/popup.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TutoDashboardValues():
    def __init__(self):
        class TutoDashboardStrings():
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
                
        class TutoDashboardStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.hello = "background-color:#d9d9d9;border-radius: 5px;font-size: 18pt; font-weight:40; color: #7c7c7c;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_TUTODASHBOARD_VALUES
        self.strings = TutoDashboardStrings()
        self.styles = TutoDashboardStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_TUTODASHBOARD_RESOURCES
    images = TutoDashboardImages()
    layouts = TutoDashboardLayouts()
    values = TutoDashboardValues()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file