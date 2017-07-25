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

DIR_TRAJECTORYXML_RESOURCES = os.path.join(get_pkg_dir('trajectory_xml'),'resources')
DIR_TRAJECTORYXML_IMAGES = DIR_TRAJECTORYXML_RESOURCES+'/images'
DIR_TRAJECTORYXML_LAYOUTS = DIR_TRAJECTORYXML_RESOURCES+'/layouts'
DIR_TRAJECTORYXML_SOUND = DIR_TRAJECTORYXML_RESOURCES+'/sound'
DIR_TRAJECTORYXML_VALUES = DIR_TRAJECTORYXML_RESOURCES+'/values'

########################################
# Class(ies) declaration
########################################

class TrajectoryXmlImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TRAJECTORYXML_IMAGES
        self.ic_launch = DIR_TRAJECTORYXML_IMAGES+'/ic_launch.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TrajectoryXmlLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TRAJECTORYXML_LAYOUTS
        self.mainwindow = DIR_TRAJECTORYXML_LAYOUTS+'/mainwindow.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TrajectoryXmlSound():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_TRAJECTORYXML_SOUND
        self.beep08b = DIR_TRAJECTORYXML_SOUND+'/beep-08b.wav'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class TrajectoryXmlValues():
    def __init__(self):
        class TrajectoryXmlStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def title(self, lng="en"):
                if lng == "en":
                    return "Trajectory LWR Recorder".decode('utf-8')
                elif lng == "fr":
                    return "Enregistement de Trajectoires XML".decode('utf-8')
                else:
                    return "Trajectory LWR Recorder".decode('utf-8')
                
        class TrajectoryXmlStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_TRAJECTORYXML_VALUES
        self.strings = TrajectoryXmlStrings()
        self.styles = TrajectoryXmlStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_TRAJECTORYXML_RESOURCES
    images = TrajectoryXmlImages()
    layouts = TrajectoryXmlLayouts()
    sound = TrajectoryXmlSound()
    values = TrajectoryXmlValues()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file