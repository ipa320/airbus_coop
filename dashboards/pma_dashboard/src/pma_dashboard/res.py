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

DIR_PMADASHBOARD_RESOURCES = os.path.join(get_pkg_dir('pma_dashboard'),'resources')
DIR_PMADASHBOARD_IMAGES = DIR_PMADASHBOARD_RESOURCES+'/images'
DIR_PMADASHBOARD_LAYOUTS = DIR_PMADASHBOARD_RESOURCES+'/layouts'
DIR_PMADASHBOARD_VALUES = DIR_PMADASHBOARD_RESOURCES+'/values'

########################################
# Class(ies) declaration
########################################

class PmaDashboardImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_PMADASHBOARD_IMAGES
        self.pma_error = DIR_PMADASHBOARD_IMAGES+'/pma_error.png'
        self.pma_running = DIR_PMADASHBOARD_IMAGES+'/pma_running.png'
        self.pma_pending = DIR_PMADASHBOARD_IMAGES+'/pma_pending.png'
        self.pma1 = DIR_PMADASHBOARD_IMAGES+'/pma1.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class PmaDashboardLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_PMADASHBOARD_LAYOUTS
        self.pma1_popup = DIR_PMADASHBOARD_LAYOUTS+'/pma1_popup.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class PmaDashboardValues():
    def __init__(self):
        class PmaDashboardStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def title(self, lng="en"):
                if lng == "en":
                    return "PMA dashboard".decode('utf-8')
                elif lng == "fr":
                    return "PMA dashboard".decode('utf-8')
                else:
                    return "PMA dashboard".decode('utf-8')
                
        class PmaDashboardStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.preempt = "QPushButton {background-color:rgb(255,0,0);border: 1px solid black;border-radius: 5px;}"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_PMADASHBOARD_VALUES
        self.strings = PmaDashboardStrings()
        self.styles = PmaDashboardStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_PMADASHBOARD_RESOURCES
    images = PmaDashboardImages()
    layouts = PmaDashboardLayouts()
    values = PmaDashboardValues()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file