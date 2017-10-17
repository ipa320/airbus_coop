#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2015 Airbus
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


########################################
# Module(s) declaration
########################################

import rospy
import os
from roslib.packages import get_pkg_dir
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi

########################################
# Constante(s) and Variable(s) declaration
########################################

DIR_PLUGINNODEMANAGER_RESOURCES = os.path.join(get_pkg_dir('airbus_plugin_node_manager'),'resources')
DIR_PLUGINNODEMANAGER_IMAGES = DIR_PLUGINNODEMANAGER_RESOURCES+'/images'
DIR_PLUGINNODEMANAGER_LAYOUTS = DIR_PLUGINNODEMANAGER_RESOURCES+'/layouts'
DIR_PLUGINNODEMANAGER_VALUES = DIR_PLUGINNODEMANAGER_RESOURCES+'/values'

########################################
# Class(ies) declaration
########################################

class PluginNodeManagerImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_PLUGINNODEMANAGER_IMAGES
        self.icon_launch_ = DIR_PLUGINNODEMANAGER_IMAGES+'/icon_launch_.png'
        self.stop = DIR_PLUGINNODEMANAGER_IMAGES+'/stop.png'
        self.icon_launch_old = DIR_PLUGINNODEMANAGER_IMAGES+'/icon_launch_old.png'
        self.start = DIR_PLUGINNODEMANAGER_IMAGES+'/start.png'
        self.launch = DIR_PLUGINNODEMANAGER_IMAGES+'/launch.png'
        self.icon_node_manager = DIR_PLUGINNODEMANAGER_IMAGES+'/icon_node_manager.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class PluginNodeManagerLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_PLUGINNODEMANAGER_LAYOUTS
        self.mainwindow = DIR_PLUGINNODEMANAGER_LAYOUTS+'/mainwindow.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class PluginNodeManagerValues():
    def __init__(self):
        class PluginNodeManagerStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def nodes_manager_title(self, lng="en"):
                if lng == "en":
                    return "Nodes manager".decode('utf-8')
                elif lng == "fr":
                    return "Gestionnaire de noeuds".decode('utf-8')
                else:
                    return "Nodes manager".decode('utf-8')
                
            def cleanup(self, lng="en"):
                if lng == "en":
                    return "Cleanup".decode('utf-8')
                elif lng == "fr":
                    return "Nettoyer".decode('utf-8')
                else:
                    return "Cleanup".decode('utf-8')
                
            def machine(self, lng="en"):
                if lng == "en":
                    return "Machine".decode('utf-8')
                elif lng == "fr":
                    return "Machine".decode('utf-8')
                else:
                    return "Machine".decode('utf-8')
                
            def node(self, lng="en"):
                if lng == "en":
                    return "Node".decode('utf-8')
                elif lng == "fr":
                    return "Noeud".decode('utf-8')
                else:
                    return "Node".decode('utf-8')
                
            def status(self, lng="en"):
                if lng == "en":
                    return "Status".decode('utf-8')
                elif lng == "fr":
                    return "Statut".decode('utf-8')
                else:
                    return "Status".decode('utf-8')
                
            def ping(self, lng="en"):
                if lng == "en":
                    return "Ping (ms)".decode('utf-8')
                elif lng == "fr":
                    return "Ping (ms)".decode('utf-8')
                else:
                    return "Ping (ms)".decode('utf-8')
                
            def start_stop(self, lng="en"):
                if lng == "en":
                    return "Start/Stop".decode('utf-8')
                elif lng == "fr":
                    return "Démarrer/Arrêtez".decode('utf-8')
                else:
                    return "Start/Stop".decode('utf-8')
                
        class PluginNodeManagerStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.label = "background-color:#d9d9d9;border-radius: 5px;font-size: 14pt;color: #7c7c7c;"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_PLUGINNODEMANAGER_VALUES
        self.strings = PluginNodeManagerStrings()
        self.styles = PluginNodeManagerStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_PLUGINNODEMANAGER_RESOURCES
    images = PluginNodeManagerImages()
    layouts = PluginNodeManagerLayouts()
    values = PluginNodeManagerValues()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file
