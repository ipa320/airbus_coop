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

DIR_SARALEARNINGPLUGIN_RESOURCES = os.path.join(get_pkg_dir('sara_learning_plugin'),'resources')
DIR_SARALEARNINGPLUGIN_SOUND = DIR_SARALEARNINGPLUGIN_RESOURCES+'/sound'
DIR_SARALEARNINGPLUGIN_VALUES = DIR_SARALEARNINGPLUGIN_RESOURCES+'/values'
DIR_SARALEARNINGPLUGIN_IMAGES = DIR_SARALEARNINGPLUGIN_RESOURCES+'/images'
DIR_SARALEARNINGPLUGIN_LAYOUTS = DIR_SARALEARNINGPLUGIN_RESOURCES+'/layouts'

########################################
# Class(ies) declaration
########################################

class SaraLearningPluginSound():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_SARALEARNINGPLUGIN_SOUND
        self.beep08b = DIR_SARALEARNINGPLUGIN_SOUND+'/beep-08b.wav'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class SaraLearningPluginValues():
    def __init__(self):
        class SaraLearningPluginStrings():
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
                
        class SaraLearningPluginStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.hello = "background-color:#d9d9d9;border-radius: 5px;font-size: 18pt; font-weight:40; color: #7c7c7c;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_SARALEARNINGPLUGIN_VALUES
        self.strings = SaraLearningPluginStrings()
        self.styles = SaraLearningPluginStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class SaraLearningPluginImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_SARALEARNINGPLUGIN_IMAGES
        self.ic_launcher = DIR_SARALEARNINGPLUGIN_IMAGES+'/ic_launcher.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class SaraLearningPluginLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_SARALEARNINGPLUGIN_LAYOUTS
        self.mainwindow = DIR_SARALEARNINGPLUGIN_LAYOUTS+'/mainwindow.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_SARALEARNINGPLUGIN_RESOURCES
    sound = SaraLearningPluginSound()
    values = SaraLearningPluginValues()
    images = SaraLearningPluginImages()
    layouts = SaraLearningPluginLayouts()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file