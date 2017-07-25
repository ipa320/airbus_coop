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

DIR_UR10PLUGIN_RESOURCES = os.path.join(get_pkg_dir('ur10_plugin'),'resources')
DIR_UR10PLUGIN_IMAGES = DIR_UR10PLUGIN_RESOURCES+'/images'
DIR_UR10PLUGIN_VALUES = DIR_UR10PLUGIN_RESOURCES+'/values'
DIR_UR10PLUGIN_LAYOUTS = DIR_UR10PLUGIN_RESOURCES+'/layouts'
DIR_UR10PLUGIN_YAMLS = DIR_UR10PLUGIN_RESOURCES+'/yamls'

########################################
# Class(ies) declaration
########################################

class Ur10PluginImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_UR10PLUGIN_IMAGES
        self.ur_product_ur5 = DIR_UR10PLUGIN_IMAGES+'/ur_product_ur5.png'
        self.ic_launcher = DIR_UR10PLUGIN_IMAGES+'/ic_launcher.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class Ur10PluginValues():
    def __init__(self):
        class Ur10PluginStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.hello = "background-color:#d9d9d9;border-radius: 5px;font-size: 18pt; font-weight:40; color: #7c7c7c;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        class Ur10PluginStrings():
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
        self.dir = DIR_UR10PLUGIN_VALUES
        self.styles = Ur10PluginStyles()
        self.strings = Ur10PluginStrings()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class Ur10PluginLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_UR10PLUGIN_LAYOUTS
        self.mainwindow = DIR_UR10PLUGIN_LAYOUTS+'/mainwindow.ui'
        self.cart_control = DIR_UR10PLUGIN_LAYOUTS+'/cart_control.ui'
        self.est = DIR_UR10PLUGIN_LAYOUTS+'/est.py'
        self.test = DIR_UR10PLUGIN_LAYOUTS+'/test.cpp'
        self.test = DIR_UR10PLUGIN_LAYOUTS+'/test.h'
        self.joint_control = DIR_UR10PLUGIN_LAYOUTS+'/joint_control.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class Ur10PluginYamls():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_UR10PLUGIN_YAMLS
        self.robot_arm = DIR_UR10PLUGIN_YAMLS+'/robot_arm.rviz'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None
        
class R:
    DIR = DIR_UR10PLUGIN_RESOURCES
    images = Ur10PluginImages()
    yamls = Ur10PluginYamls()
    values = Ur10PluginValues()
    layouts = Ur10PluginLayouts()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file