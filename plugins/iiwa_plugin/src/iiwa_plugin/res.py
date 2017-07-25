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

DIR_IIWAPLUGIN_RESOURCES = os.path.join(get_pkg_dir('iiwa_plugin'),'resources')
DIR_IIWAPLUGIN_YAMLS = DIR_IIWAPLUGIN_RESOURCES+'/yamls'
DIR_IIWAPLUGIN_VALUES = DIR_IIWAPLUGIN_RESOURCES+'/values'
DIR_IIWAPLUGIN_IMAGES = DIR_IIWAPLUGIN_RESOURCES+'/images'
DIR_IIWAPLUGIN_LAYOUTS = DIR_IIWAPLUGIN_RESOURCES+'/layouts'

########################################
# Class(ies) declaration
########################################

class IiwaPluginYamls():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_IIWAPLUGIN_YAMLS
        self.robot_arm = DIR_IIWAPLUGIN_YAMLS+'/robot_arm.rviz'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class IiwaPluginValues():
    def __init__(self):
        class IiwaPluginStrings():
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
                
        class IiwaPluginStyles():
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
        self.dir = DIR_IIWAPLUGIN_VALUES
        self.strings = IiwaPluginStrings()
        self.styles = IiwaPluginStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class IiwaPluginImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_IIWAPLUGIN_IMAGES
        self.attcah_tool = DIR_IIWAPLUGIN_IMAGES+'/attcah_tool.png'
        self.arrow_right = DIR_IIWAPLUGIN_IMAGES+'/arrow_right.png'
        self.min_arrow_left = DIR_IIWAPLUGIN_IMAGES+'/min_arrow_left.png'
        self.arrow_down = DIR_IIWAPLUGIN_IMAGES+'/arrow_down.png'
        self.arrow_left = DIR_IIWAPLUGIN_IMAGES+'/arrow_left.png'
        self.min_arrow_right = DIR_IIWAPLUGIN_IMAGES+'/min_arrow_right.png'
        self.blueon48 = DIR_IIWAPLUGIN_IMAGES+'/blue-on-48.png'
        self.robot_view_1 = DIR_IIWAPLUGIN_IMAGES+'/robot_view_1.png'
        self.robot_view_4 = DIR_IIWAPLUGIN_IMAGES+'/robot_view_4.png'
        self.ico_minus = DIR_IIWAPLUGIN_IMAGES+'/ico_minus.png'
        self.robot_view_3 = DIR_IIWAPLUGIN_IMAGES+'/robot_view_3.png'
        self.ico_plus = DIR_IIWAPLUGIN_IMAGES+'/ico_plus.png'
        self.robot_view_2 = DIR_IIWAPLUGIN_IMAGES+'/robot_view_2.png'
        self.redon48 = DIR_IIWAPLUGIN_IMAGES+'/red-on-48.png'
        self.ico_robot_arm = DIR_IIWAPLUGIN_IMAGES+'/ico_robot_arm.png'
        self.iiwa_frame = DIR_IIWAPLUGIN_IMAGES+'/iiwa_frame.png'
        self.arrow_up = DIR_IIWAPLUGIN_IMAGES+'/arrow_up.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class IiwaPluginLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_IIWAPLUGIN_LAYOUTS
        self.robot_axis = DIR_IIWAPLUGIN_LAYOUTS+'/robot_axis.ui'
        self.cart_control = DIR_IIWAPLUGIN_LAYOUTS+'/cart_control.ui'
        self.io_control_2 = DIR_IIWAPLUGIN_LAYOUTS+'/io_control_2.ui'
        self.mainwidow = DIR_IIWAPLUGIN_LAYOUTS+'/mainwidow.ui'
        self.io_control = DIR_IIWAPLUGIN_LAYOUTS+'/io_control.ui'
        self.joint_control = DIR_IIWAPLUGIN_LAYOUTS+'/joint_control.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_IIWAPLUGIN_RESOURCES
    yamls = IiwaPluginYamls()
    values = IiwaPluginValues()
    images = IiwaPluginImages()
    layouts = IiwaPluginLayouts()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file