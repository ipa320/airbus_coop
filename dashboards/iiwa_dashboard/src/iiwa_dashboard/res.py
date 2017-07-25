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

DIR_IIWADASHBOARD_RESOURCES = os.path.join(get_pkg_dir('iiwa_dashboard'),'resources')
DIR_IIWADASHBOARD_IMAGES = DIR_IIWADASHBOARD_RESOURCES+'/images'
DIR_IIWADASHBOARD_LAYOUTS = DIR_IIWADASHBOARD_RESOURCES+'/layouts'
DIR_IIWADASHBOARD_VALUES = DIR_IIWADASHBOARD_RESOURCES+'/values'

########################################
# Class(ies) declaration
########################################

class IiwaDashboardImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_IIWADASHBOARD_IMAGES
        self.comm_green = DIR_IIWADASHBOARD_IMAGES+'/comm_green.png'
        self.error_green = DIR_IIWADASHBOARD_IMAGES+'/error_green.png'
        self.debug_level = DIR_IIWADASHBOARD_IMAGES+'/debug_level.png'
        self.led_on = DIR_IIWADASHBOARD_IMAGES+'/led_on.png'
        self.comm_grey = DIR_IIWADASHBOARD_IMAGES+'/comm_grey.png'
        self.led_off = DIR_IIWADASHBOARD_IMAGES+'/led_off.png'
        self.error_red = DIR_IIWADASHBOARD_IMAGES+'/error_red.png'
        self.error_grey = DIR_IIWADASHBOARD_IMAGES+'/error_grey.png'
        self.icon_robot = DIR_IIWADASHBOARD_IMAGES+'/icon_robot.png'
        self.base = DIR_IIWADASHBOARD_IMAGES+'/base.png'
        self.collision = DIR_IIWADASHBOARD_IMAGES+'/collision.png'
        self.icon_robot_running = DIR_IIWADASHBOARD_IMAGES+'/icon_robot_running.png'
        self.icon_robot_unknow = DIR_IIWADASHBOARD_IMAGES+'/icon_robot_unknow.png'
        self.comm_red = DIR_IIWADASHBOARD_IMAGES+'/comm_red.png'
        self.icon_robot_in_default = DIR_IIWADASHBOARD_IMAGES+'/icon_robot_in_default.png'
        self.icon_robot_e_stopped = DIR_IIWADASHBOARD_IMAGES+'/icon_robot_e_stopped.png'
        self.icon_robot_collision = DIR_IIWADASHBOARD_IMAGES+'/icon_robot_collision.png'
        self.power_green = DIR_IIWADASHBOARD_IMAGES+'/power_green.png'
        self.power_grey = DIR_IIWADASHBOARD_IMAGES+'/power_grey.png'
        self.connection = DIR_IIWADASHBOARD_IMAGES+'/connection.png'
        self.tool = DIR_IIWADASHBOARD_IMAGES+'/tool.png'
        self.power_red = DIR_IIWADASHBOARD_IMAGES+'/power_red.png'
        self.velocity = DIR_IIWADASHBOARD_IMAGES+'/velocity.png'
        self.icon_robot_error = DIR_IIWADASHBOARD_IMAGES+'/icon_robot_error.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class IiwaDashboardLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_IIWADASHBOARD_LAYOUTS
        self.robot = DIR_IIWADASHBOARD_LAYOUTS+'/robot.ui'
        self.popup = DIR_IIWADASHBOARD_LAYOUTS+'/popup.ui'
        self.defaults = DIR_IIWADASHBOARD_LAYOUTS+'/defaults.ui'
        self.robot_status_popup = DIR_IIWADASHBOARD_LAYOUTS+'/robot_status_popup.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class IiwaDashboardValues():
    def __init__(self):
        class IiwaDashboardStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def title(self, lng="en"):
                if lng == "en":
                    return "IIWA Dashboard".decode('utf-8')
                elif lng == "fr":
                    return "IIWA Dashboard".decode('utf-8')
                else:
                    return "IIWA Dashboard".decode('utf-8')
                
            def robot_in_error(self, lng="en"):
                if lng == "en":
                    return "The robot is in error !".decode('utf-8')
                elif lng == "fr":
                    return "Le robot est en erreur !".decode('utf-8')
                else:
                    return "The robot is in error !".decode('utf-8')
                
            def acquit_timeout_exceeded(self, lng="en"):
                if lng == "en":
                    return "Timeout from acquit default exceeded !".decode('utf-8')
                elif lng == "fr":
                    return "Délai d'acquittement dépassé !".decode('utf-8')
                else:
                    return "Timeout from acquit default exceeded !".decode('utf-8')
                
            def unknown(self, lng="en"):
                if lng == "en":
                    return "Unknown".decode('utf-8')
                elif lng == "fr":
                    return "Inconnu".decode('utf-8')
                else:
                    return "Unknown".decode('utf-8')
                
        class IiwaDashboardStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.default_dialog = "QDialog{background-color:#ffffff;border: 5px solid red;}QLabel{background-color: transparent;border: none};"
                self.acquit_button = "background-color:qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #dc7575, stop: 1 #8f0000);color:#ffffff;font: 63 16pt;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_IIWADASHBOARD_VALUES
        self.strings = IiwaDashboardStrings()
        self.styles = IiwaDashboardStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_IIWADASHBOARD_RESOURCES
    images = IiwaDashboardImages()
    layouts = IiwaDashboardLayouts()
    values = IiwaDashboardValues()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file