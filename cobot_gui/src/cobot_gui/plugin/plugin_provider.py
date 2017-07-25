#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : plugin_provider.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

import rospy
import uuid
import os
import sys

import __builtin__
import traceback
from xml.etree import ElementTree
from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from pyqt_agi_extend.QtAgiCore import QAgiPackages, get_pkg_dir_from_prefix
from pyqt_agi_extend.QtAgiGui import QAgiPopup

from cobot_gui.util import Parameters, CobotGuiException

## @package: plugin_provider
## @version 4.0
## @author  Matignon Martin
## @date    Last modified 28/02/2014
## @class PluginProvider
## @brief Class for load Python plugin package.
class PluginProvider:
    
    """
    PluginProvider interacts with ros plugin package. The first is its
    import plugin, and the second is the set plugin configuration which
    it reads.
    """
    
    PLUGIN_SOURCES_LOCATION = 'src'
    
    def __init__(self, parent, xml_register_dir):
        """! The constructor."""
        
        self._context = parent.getContext()
        
        #Check dir 
        if not os.path.isfile(xml_register_dir):
            raise CobotGuiException('Plugin register file "%s" in package "cobot_gui" not found'
                                     %(xml_register_dir))
        
        #Parse xml file
        try:
            self._plugin_register = ElementTree.parse(xml_register_dir)
        except Exception as e:
            raise CobotGuiException(str(e))
        
    def getPkgByName(self, name):
        
        root = self._plugin_register.getroot()
        
        #Find and read node label
        plugin_desc = root.find('./plugin[@label="%s"]'%name)
        
        if plugin_desc is None:
            raise CobotGuiException('Cannot found package from plugin named "%ss"'%name)
        
        return plugin_desc.attrib['package']
    
    def getInstance(self, plugin_name, plugin_node=None):
        """! Load Python package and provide plugin instance.
        @param plugin_name: plugin name.
        @type plugin_name: String.
        
        @param plugin_node: plugin xml element tree.
        @type plugin_node: Element.
        
        @return plugin_instance: plugin instance.
        @type plugin_instance: Plugin.
        """
        
        plugin_pkg_name = None
        
        try:
            
            plugin_pkg_name = self.getPkgByName(plugin_name)
            
        except Exception as ex:
            self._context.getLogger().err(str(ex))
            return None
        
        plugin_dir = get_pkg_dir(plugin_pkg_name)
        
        plugin_descriptor_file = os.path.join(plugin_dir,"plugin_descriptor.xml")
        
        if not os.path.isfile(plugin_descriptor_file):
            self._context.getLogger().err('Cannot found plugin_descriptor.xml into plugin %s'%plugin_name)
            return None
        
        plugin_descriptor_root = ElementTree.parse(plugin_descriptor_file).getroot()
        
        plugin_import = plugin_descriptor_root.find('import')
         
        plugin_module_path = plugin_import.attrib['module']
        plugin_class_name = plugin_import.attrib['class']
        
        sys.path.append(os.path.join(plugin_dir,self.PLUGIN_SOURCES_LOCATION))
        
        plugin_class_ref = None
        
        try:
            #Import plugin package module
            module = __builtin__.__import__(plugin_module_path,
                                            fromlist=[plugin_class_name],
                                            level=0)
            
        except Exception as ex:
            self._context.getLogger().err("Cannot import plugin '%s' !\n%s"%(plugin_name, str(ex)))
            return None
        
        #Get referance to plugin class
        plugin_class_ref = getattr(module, plugin_class_name)
        
        if plugin_class_ref is None:
            self._context.getLogger().err("Cannot found plugin class '%s' !"%plugin_class_name)
            return None
        
        plugin_instance = plugin_class_ref(self._context)
        
        plugin_params = PluginProvider.getParameters(plugin_descriptor_root, plugin_node)
        
        plugin_instance.setup(plugin_descriptor_root, plugin_params)
        
        return plugin_instance
    
    @staticmethod
    def getParameters(plugin_descriptor, plugin_node):
        
        parameters = Parameters()
        
        # Try to provide plugin parameters in plugin_descriptor.xml
        descriptor_params = plugin_descriptor.find("setup/parameters")
        
        # Add parameters, if parameters found in plugin_descriptor.xml
        if descriptor_params is not None:
            for param in descriptor_params:
                # Append parameters
                parameters.putParam(param.attrib['name'], param.attrib['value'])
                
        if plugin_node is not None:
            # Check if parameters remapped in cobot_gui config launch
            if plugin_node.find("param") is not None:
                
                for param in plugin_node.iter('param'):
                    # Update or append parameters
                    parameters.putParam(param.attrib['name'], param.attrib['value'])
        
        return parameters
    
class PluginsGroupPopup(QAgiPopup):
    
    def __init__(self, parent):
        QAgiPopup.__init__(self, parent)
        
        self._context = parent.getContext()
        
        self.setAttribute(Qt.WA_TranslucentBackground)
        self.setFixedWidth(100)
        self.setRelativePosition(QAgiPopup.TopLeft, QAgiPopup.TopRight)
        
        self._launchers_layout = QVBoxLayout(self)
        self._launchers_layout.setContentsMargins(2, 2, 2, 2)
        self._launchers_layout.setSpacing(15)
    
    def setupLaunchers(self, launchers):
        
        for launcher in launchers:
            self.connect(launcher, SIGNAL('clicked()'), self.close)
            self._launchers_layout.addWidget(launcher)
        
class PluginsGroup(QPushButton):
    
    def __init__(self, parent, xgroup):
        
        QPushButton.__init__(self, parent)
        
        self.setFocusPolicy(Qt.NoFocus)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        self.setIconSize(QSize(80,80))
        self.setEnabled(False)
        
        self._context = parent.getContext()
        self._context.addUserEventListener(self.onUserChanged)
        
        self._min_access_rights = 3
        self._launchers = []
        
        self.setup(xgroup)
        
    def setup(self, xgroup):
        
        group_name = ""
        icon_path = ""
        
        try:
            group_name = xgroup.attrib['name']
            icon_path = xgroup.attrib['icon']
        except:
            self._context.getLogger().err("Not name or icon found for plugin group !")
            return
        
        icon_path = get_pkg_dir_from_prefix(icon_path)
        
        if os.path.isfile(icon_path):
            self.setIcon(QIcon(icon_path))
        else:
            self.setStyleSheet("background-color:rgba(255,0,0,80%);\
                                                 border-radius: 10px;\
                                                 font-size: 12pt;\
                                                 font-weight:60;\
                                                 color: #ffffff;")
            self.setText(group_name)
        
    def add(self, launcher):
        
        launcher.setStyleSheet("background:none;")#R.values.styles.no_background)
        
        self._launchers.append(launcher)
        
        if launcher.getAccessRights() < self._min_access_rights:
            self._min_access_rights = launcher.getAccessRights()
        
    def getContext(self):
        self._context
        
    def onUserChanged(self, user):
        
        if user.getUserPrivilege() < self._min_access_rights:
            self.setEnabled(False)
        else:
            self.setEnabled(True)
    
    def mousePressEvent(self, event):
        
        popup = PluginsGroupPopup(self)
        popup.setupLaunchers(self._launchers)
        
        popup.show_()
        
##Unittest
if __name__ == "__main__":
    from python_qt_binding.QtGui import *
    from python_qt_binding.QtCore import *
    from cobot_gui.context import Context
    
    rospy.init_node('plugin_privider_test')
    
    a = QApplication(sys.argv)
    utt_appli = QMainWindow()

    context = Context(utt_appli)
    
    provider = PluginProvider(context, "/home/nhg/AIRBUS/airbus_coop/src/airbus_coop/src/gui/plugins/plugins_register.xml")
    
    plugin = provider.getPluginInstance("SSM")
    
    utt_appli.setCentralWidget(plugin)
    
    plugin.onStart()
    
    utt_appli.show()
    a.exec_()
    
    

#End of file

