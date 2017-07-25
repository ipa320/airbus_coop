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

DIR_PLUGINHELICOPTERE_RESOURCES = os.path.join(get_pkg_dir('plugin_helicoptere'),'resources')
DIR_PLUGINHELICOPTERE_IMAGES = DIR_PLUGINHELICOPTERE_RESOURCES+'/images'
DIR_PLUGINHELICOPTERE_LAYOUTS = DIR_PLUGINHELICOPTERE_RESOURCES+'/layouts'
DIR_PLUGINHELICOPTERE_YAMLS = DIR_PLUGINHELICOPTERE_RESOURCES+'/yamls'
DIR_PLUGINHELICOPTERE_VALUES = DIR_PLUGINHELICOPTERE_RESOURCES+'/values'

########################################
# Class(ies) declaration
########################################

class PluginHelicoptereImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_PLUGINHELICOPTERE_IMAGES
        self.image_6 = DIR_PLUGINHELICOPTERE_IMAGES+'/image_6.png'
        self.image_1 = DIR_PLUGINHELICOPTERE_IMAGES+'/image_1.png'
        self.image_7 = DIR_PLUGINHELICOPTERE_IMAGES+'/image_7.png'
        self.image_4 = DIR_PLUGINHELICOPTERE_IMAGES+'/image_4.png'
        self.image_3 = DIR_PLUGINHELICOPTERE_IMAGES+'/image_3.png'
        self.image_2 = DIR_PLUGINHELICOPTERE_IMAGES+'/image_2.png'
        self.launcher = DIR_PLUGINHELICOPTERE_IMAGES+'/launcher.png'
        self.image_8 = DIR_PLUGINHELICOPTERE_IMAGES+'/image_8.png'
        self.image_5 = DIR_PLUGINHELICOPTERE_IMAGES+'/image_5.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class PluginHelicoptereLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_PLUGINHELICOPTERE_LAYOUTS
        self.mainwidow = DIR_PLUGINHELICOPTERE_LAYOUTS+'/mainwidow.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class PluginHelicoptereYamls():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_PLUGINHELICOPTERE_YAMLS
        self.robot_arm = DIR_PLUGINHELICOPTERE_YAMLS+'/robot_arm.yaml'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class PluginHelicoptereValues():
    def __init__(self):
        class PluginHelicoptereStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def title(self, lng="en"):
                if lng == "en":
                    return "Please select up to three items and validate".decode('utf-8')
                elif lng == "fr":
                    return "Veuillez sélectionner jusqu’à trois objets puis valider".decode('utf-8')
                else:
                    return "Please select up to three items and validate".decode('utf-8')
                
            def no_images_selected(self, lng="en"):
                if lng == "en":
                    return "Please select at least one image".decode('utf-8')
                elif lng == "fr":
                    return "Veuillez sélectionner au moins une image".decode('utf-8')
                else:
                    return "Please select at least one image".decode('utf-8')
                
            def validate(self, lng="en"):
                if lng == "en":
                    return "Validate".decode('utf-8')
                elif lng == "fr":
                    return "Valider".decode('utf-8')
                else:
                    return "Validate".decode('utf-8')
                
            def max_images_reached(self, lng="en"):
                if lng == "en":
                    return "You have already selected 3 objects".decode('utf-8')
                elif lng == "fr":
                    return "vous avez déjà sélectionné 3 objets".decode('utf-8')
                else:
                    return "You have already selected 3 objects".decode('utf-8')
                
        class PluginHelicoptereStyles():
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
        self.dir = DIR_PLUGINHELICOPTERE_VALUES
        self.strings = PluginHelicoptereStrings()
        self.styles = PluginHelicoptereStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_PLUGINHELICOPTERE_RESOURCES
    images = PluginHelicoptereImages()
    layouts = PluginHelicoptereLayouts()
    yamls = PluginHelicoptereYamls()
    values = PluginHelicoptereValues()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file