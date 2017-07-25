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

DIR_AVANCEMENTSTATIONPLUGIN_RESOURCES = os.path.join(get_pkg_dir('avancement_station_plugin'),'resources')
DIR_AVANCEMENTSTATIONPLUGIN_IMAGES = DIR_AVANCEMENTSTATIONPLUGIN_RESOURCES+'/images'
DIR_AVANCEMENTSTATIONPLUGIN_VALUES = DIR_AVANCEMENTSTATIONPLUGIN_RESOURCES+'/values'
DIR_AVANCEMENTSTATIONPLUGIN_LAYOUTS = DIR_AVANCEMENTSTATIONPLUGIN_RESOURCES+'/layouts'

########################################
# Class(ies) declaration
########################################

class AvancementStationPluginImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_AVANCEMENTSTATIONPLUGIN_IMAGES
        self.ic_launcher = DIR_AVANCEMENTSTATIONPLUGIN_IMAGES+'/ic_launcher.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class AvancementStationPluginValues():
    def __init__(self):
        class AvancementStationPluginStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.hello = "background-color:#d9d9d9;border-radius: 5px;font-size: 18pt; font-weight:40; color: #7c7c7c;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        class AvancementStationPluginStrings():
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
                
        self.uuid = self.__class__.__name__
        self.dir = DIR_AVANCEMENTSTATIONPLUGIN_VALUES
        self.styles = AvancementStationPluginStyles()
        self.strings = AvancementStationPluginStrings()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class AvancementStationPluginLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_AVANCEMENTSTATIONPLUGIN_LAYOUTS
        self.mainwindow = DIR_AVANCEMENTSTATIONPLUGIN_LAYOUTS+'/mainwindow.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_AVANCEMENTSTATIONPLUGIN_RESOURCES
    images = AvancementStationPluginImages()
    values = AvancementStationPluginValues()
    layouts = AvancementStationPluginLayouts()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file