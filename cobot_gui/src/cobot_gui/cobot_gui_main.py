#!/usr/bin/env python
# -*- coding: utf-8 -*-
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : cobot_gui_main.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

## @package: cobot_gui_main
## @version 4.0
## @author  Matignon Martin
## @date    Last modified 22/08/2014

import rospy
import os
import time

from roslib.packages import get_pkg_dir
from xml.etree import ElementTree

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiCore import get_pkg_dir_from_prefix

from context import Context

from plugin.plugin_provider import PluginProvider, PluginsGroup
from dashboard import DashboardProvider

from util import CobotGuiException
from emergency import EmergencyStopButton, EmergencyStopState
from account import User, \
                    Privilege, \
                    LoginDialog, \
                    UserAccountsWidget
                    
# Minumum ui tools
from cobot_gui.control_mode import ControlModeWidget, ControlMode
from cobot_gui.translator import TranslatorUi
from cobot_gui.timestamp import Timestamp

from alarm import AlarmManagerWidget, Alarm

from std_msgs.msg import String

from cobot_gui.res import R

class CobotGuiSplash(QSplashScreen):
    
    def __init__(self):
        QSplashScreen.__init__(self)
        
        # Extend the widget with all attributes and children from UI file
        loadUi(R.layouts.welcome, self)
        
        self.setPixmap(R.getPixmapById("wellcome_background").scaled(600, 400,
                                       Qt.KeepAspectRatio,
                                       Qt.SmoothTransformation))
        
        self.loading_progess.setText("Loading ...")
        
    def start(self):
        self.show()
        self.showMessage(" ")
        
    def update(self, txt):
        self.loading_progess.setText("Loading %s ..."%txt)
        self.showMessage(" ")

## @class CobotGuiMain
## @brief Setup all graphics components (window, plugins, dashboard).
class CobotGuiMain(QWidget):
    """! CobotGuiMain class inherit QWidget.
    This class setup all graphics components difinit on your config file:
     - Setup default rules : Language, size, account, ...
     - Setup dashboard : load widgets on dashboard registered in section <dashboard>,
     - Setup launchers : load plugins on launchers registered in section <launcher>.
    """
    APP_MODE_RELEASE = 'release'
    APP_MODE_DEBUG   = 'debug'
    
    def __init__(self, splash):
        """! The constructor.
        @param config: Config file (*.xml).
        """
        QWidget.__init__(self)
        
        loadUi(R.layouts.mainwindow, self)
        self.setAttribute(Qt.WA_AcceptTouchEvents)
        
        self._splash = splash
        self.display_mode = ''
        # Default components size
        self._launcher_width = 100
        self._dashboard_height = 80
        
        # Plugin loaded by default on app boot.
        self._default_view = None
        # Link current plugin loaded on viewer
        self._current_view = None
        # Container for plugins group instance
        self._plugins_group_list = []
        
        self._context = Context(self)
        
        self._context.addViewManagerEventListner(self.onManageView)
        self._context.addUserConnectionEventListener(self.onUserChanged)
        self._context.addControlModeEventListener(self.onControlModeChanged)
        self._context.addLanguageEventListner(self.onTranslate)
        
        self._context.addEmergencyStopEventListner(self.onEmergencyStop)
        
        self.setupMinimumTools()
        
    def setupMinimumTools(self):
        
        self.control_mode_widget = ControlModeWidget(self._context)
        self.ctrl_layout.addWidget(self.control_mode_widget)
        
        self.user_account = UserAccountsWidget(self._context)
        self.user_account.onCreate(None)
        self.user_layout.addWidget(self.user_account)
        
        self.timestamp_widget = Timestamp(self._context)
        self.timestamp_widget.onCreate(None)
        self.user_layout.addWidget(self.timestamp_widget)
        
        self.translator_widget = TranslatorUi(self._context)
        self.user_layout.addWidget(self.translator_widget)
        
        self.emergency_stop = EmergencyStopButton(self._context)
        self.interruption_layout.addWidget(self.emergency_stop)
        
        self.alarms_manager = AlarmManagerWidget(self._context)
        self.alarm_layout.addWidget(self.alarms_manager)
        
        #Display Airbus Group logo
        self.logo_label.setPixmap(R.getPixmapById('logo_airbus_group').scaled(
                           self.logo_label.width()-2,
                           self.logo_label.height()-2,
                           Qt.KeepAspectRatio,
                           Qt.SmoothTransformation))
        
    def setupUserConfig(self, config_xml):
        """! Parser xml configuration file.
        @param config_xml: cobot_gui configuration path.
        @type config_xml: string.
        """
        
        #Check path from configuration file
        if not os.path.isfile(config_xml):
            self._context.getLogger().critical('User config file "%s" not found !'%config_xml)
        
        #Open and parse xml file
        xconfig = ElementTree.parse(config_xml).getroot()
        
        app_mode = self.APP_MODE_RELEASE
        
        try:
            app_mode = xconfig.attrib['mode'].lower()
        except:
            pass
        
        lng = Context.DEFAULT_LNG
        
        try:
            lng = xconfig.find('translate').attrib['type'].lower()
        except:
            pass
            
        xwindow = xconfig.find('window')
        
        if xwindow is None:
            self._context.getLogger().critical('Cannot found "<window>" tag into config file !')
            return
        
        try:
            self.display_mode = xwindow.attrib['display-mode'].lower()
        except:
            self.display_mode = ""
        
        #Read node window
        for node in xwindow:
            
            if node.tag == 'default-size':
                
                try:
                    width = int(node.find('width').text)
                    height = int(node.find('height').text)
                    self.resize(width, height)
                except:
                    self.resize(1920, 1080)
                    
            elif node.tag == 'header':
                self.installHeader(node)
            elif node.tag == 'launcher':
                self.installLauncher(node)
            else:
                self.getContext().getLogger().warn('Invalid tag "%s" into user configuration !'%node.tag)
        
        self.getContext().switchLanguage(lng)
        
        if app_mode == self.APP_MODE_DEBUG:
            self.getContext().switchUser(User('Airbus Group', Privilege.EXPERT))
        else:
            # Load default user none -> open login dialog
            self.getContext().switchUser(User())
            login = LoginDialog(self, False)
            QTimer.singleShot(1000, login.show)
        
    def installHeader(self, xheader):
        """! Setup all widgets on dashbord registered on config file.
        @param tree: node dashbord.
        @type tree: ElementTree.
        """
        
        for node in xheader:
            
            if node.tag == 'dashboards':
                
                register_dir = node.attrib['src']
                register_dir = get_pkg_dir_from_prefix(register_dir)
                
                if not os.path.isfile(register_dir):
                    self._context.getLogger().critical('Dashboards register file "%s" not found !'%register_dir)
                    return
                
                dashboard_provider = DashboardProvider(self, register_dir)
                
                for child in node:
                    
                    if child.tag == 'dashboard':
                        
                        dashboard_name = child.attrib['name']
                        
                        #Update splash from display the current dashboard loading
                        self._splash.update(dashboard_name)
                        
                        try:
                            dashboard = dashboard_provider.getInstance(dashboard_name, child)
                            
                            self.dashboard_layout.addWidget(dashboard)
                            
                        except Exception as ex:
                            self._context.getLogger().err('Try to provide "%s" instance failed !\n%s'
                                                          %(dashboard_name, str(ex)))
                            continue
                        
    
    def installLauncher(self, xlaunchers):
        """! Setup plugins and launcher.
        @param xlaunchers: node launcher.
        @type xlaunchers: ElementTree.
        """
        
        default_plugin_name = ""
        default_plugin = None
        control_mode = ControlMode.MANUAL
        
        try:
            default_plugin_name = xlaunchers.attrib['default-view']
        except:
            pass
        
        try:
            control_mode = ControlMode.TOLEVEL[xlaunchers.attrib['default-mode'].lower()]
        except:
            self._context.getLogger().warn("Invalid 'default-mode' attribute into config file !")
            control_mode = ControlMode.MANUAL
        
        for node in xlaunchers:
            
            if node.tag == 'plugins':
                
                plugins_register_dir = node.attrib['src']
                plugins_register_dir = get_pkg_dir_from_prefix(plugins_register_dir)
                
                if not os.path.isfile(plugins_register_dir):
                    self._context.getLogger().critical('Plugins register file "%s" not found !'%plugins_register_dir)
                    return
                
                provider = PluginProvider(self, plugins_register_dir)
                
                for xplugin in node:
                    
                    if xplugin.tag == 'plugin':
                        
                        plugin_name = xplugin.attrib['name']
                        
                        self._splash.update(plugin_name)
                        
                        try:
                            plugin = provider.getInstance(plugin_name, xplugin)
                            plugin.tryToPause()
                            
                            if plugin_name == default_plugin_name:
                                default_plugin = plugin
                            
                            self.launcher_layout.addWidget(plugin.getLauncher())
                            
                        except Exception as ex:
                            self._context.getLogger().err('Try to provide "%s" instance failed !\n%s'
                                                          %(plugin_name, str(ex)))
                            continue
                        
                    elif xplugin.tag == 'group':
                        
                        plugins_group = PluginsGroup(self, xplugin)
                        
                        for xsubplugin in xplugin:
                            
                            plugin_name = xsubplugin.attrib['name']
                            self._splash.update(plugin_name)
                             
                            try:
                                
                                plugin = provider.getInstance(plugin_name, xsubplugin)
                                plugin.tryToPause()
                                
                                plugins_group.add(plugin.getLauncher())
                                
                                if plugin_name == default_plugin_name:
                                    default_plugin = plugin
                                 
                            except Exception as ex:
                                self._context.getLogger().err('Try to provide "%s" instance failed !\n%s'
                                                              %(plugin_name, str(ex)))
                                continue
                        
                        self.launcher_layout.addWidget(plugins_group)
        
        if default_plugin is not None:
            default_plugin.onRequestDisplayView()
        
        self.control_mode_widget.setDefaultMode(control_mode)
    
    def getContext(self):
        return self._context
        
    def getDisplayMode(self):
        return self.display_mode
    
    def onManageView(self, view):
        # Sets current plugin activity to pause
        if self._current_view is not None:
            self._current_view.tryToPause()
            
        # Sets new plugin activity to resume
        view.tryToResume()
        
        if self._current_view is not None:
            # Remove current plugin view
            self.viewer.takeWidget()
            
        # Sets new plugin view on viewer
        self.viewer.setWidget(view)
        
        self._current_view = view
    
    def onUserChanged(self, user):
        
        if self._current_view is not None:
            
            if user.getUserPrivilege() == Privilege.NONE:
            #{
                self.viewer.takeWidget()
            #}
            elif self._current_view.getLauncher().getAccessRights() > user.getUserPrivilege():
            #{
                self.viewer.takeWidget()
            #}
            else:
                pass
        else:
            pass
        
    def onControlModeChanged(self, mode):
        pass
    
    def onTranslate(self, lng):
        pass
        
    def onEmergencyStop(self, state):
        """! Called when emergency stop status changed.
        @param status: emergency stop status.
        @type status: bool.
        """
        
        if state == EmergencyStopState.LOCKED:
            self.dashboard_widget.setStyleSheet(R.values.styles.background_estop_locked)
            self.logo_label.setStyleSheet(R.values.styles.background_estop_locked)
        else:
            self.dashboard_widget.setStyleSheet(R.values.styles.background_estop_unlocked)
            self.logo_label.setStyleSheet(R.values.styles.background_estop_unlocked)
    
    def resizeEvent(self, event):
        """! Resize application.
        @param event: event object.
        @type event: QEvent.
        """
        pass
    
    def shutdown(self):
        """! This methode call shutdown from all cobot_gui instances.
        """
        self._context.requestShutdown()

#End of file

