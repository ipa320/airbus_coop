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

DIR_ALDOPLUGIN_RESOURCES = os.path.join(get_pkg_dir('aldo_plugin'),'resources')
DIR_ALDOPLUGIN_YAMLS = DIR_ALDOPLUGIN_RESOURCES+'/yamls'
DIR_ALDOPLUGIN_VALUES = DIR_ALDOPLUGIN_RESOURCES+'/values'
DIR_ALDOPLUGIN_IMAGES = DIR_ALDOPLUGIN_RESOURCES+'/images'
DIR_ALDOPLUGIN_LAYOUTS = DIR_ALDOPLUGIN_RESOURCES+'/layouts'

########################################
# Class(ies) declaration
########################################

class AldoPluginYamls():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_ALDOPLUGIN_YAMLS
        self.robot_arm = DIR_ALDOPLUGIN_YAMLS+'/robot_arm.rviz'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class AldoPluginValues():
    def __init__(self):
        class AldoPluginStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def title(self, lng="en"):
                if lng == "en":
                    return "IIWA plugin".decode('utf-8')
                elif lng == "fr":
                    return "IIWA plugin".decode('utf-8')
                else:
                    return "IIWA plugin".decode('utf-8')
                
        class AldoPluginStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.io_background = "QLabel {background-color:#646464;border: 1px solid black;border-radius: 5px;}"
                self.io_on = "QLabel {background-color: qlineargradient(x1:0.5, y1:0.994318,x2:0.507752, y2:0,stop:0 #00a91c,stop:1 #78d900);border: 1px solid black;border-radius: 5px;font-size: 18pt;color: #ffffff;qproperty-alignment: AlignCenter;}"
                self.io_off = "QLabel {background-color: qlineargradient(x1:0.5, y1:0.994318, x2:0.507752, y2:0, stop:0 #005500, stop:1 #003800);border: 1px solid black;border-radius: 5px;font-size: 18pt;color: #ffffff;qproperty-alignment: AlignCenter;}"
                self.out_joint_angle = "background-color:qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ff0000, stop: 1 #ff8080); border: 2px #616763; border-radius: 5px; font-size: 18pt;  font-weight:40;  color: #000000;"
                self.joint_angle_ok = "background-color:qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #2ca1cf, stop: 1 #0482bb); border: 2px #616763; border-radius: 5px; font-size: 18pt;  font-weight:40;  color: #000000;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_ALDOPLUGIN_VALUES
        self.strings = AldoPluginStrings()
        self.styles = AldoPluginStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class AldoPluginImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_ALDOPLUGIN_IMAGES
        self.ic_launcher = DIR_ALDOPLUGIN_IMAGES+'/ic_launcher.png'
        self.ico_minus = DIR_ALDOPLUGIN_IMAGES+'/ico_minus.png'
        self.ico_plus = DIR_ALDOPLUGIN_IMAGES+'/ico_plus.png'
        self.ico_robot_arm = DIR_ALDOPLUGIN_IMAGES+'/ico_robot_arm.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class AldoPluginLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_ALDOPLUGIN_LAYOUTS
        self.robot_axis = DIR_ALDOPLUGIN_LAYOUTS+'/robot_axis.ui'
        self.cart_control = DIR_ALDOPLUGIN_LAYOUTS+'/cart_control.ui.autosave'
        self.cart_control = DIR_ALDOPLUGIN_LAYOUTS+'/cart_control.ui'
        self.joint_control = DIR_ALDOPLUGIN_LAYOUTS+'/joint_control.ui'
        self.mainwindow = DIR_ALDOPLUGIN_LAYOUTS+'/mainwindow.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_ALDOPLUGIN_RESOURCES
    yamls = AldoPluginYamls()
    values = AldoPluginValues()
    images = AldoPluginImages()
    layouts = AldoPluginLayouts()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file