#!/usr/bin/env python
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

## @package: dashboard_provider
## @version 4.0
## @author  Matignon Martin
## @date    Last modified 28/02/2014
## @class PluginProvider
## @brief Class for load Python plugin package.
class DashboardProvider:
    
    """
    PluginProvider interacts with ros plugin package. The first is its
    import plugin, and the second is the set plugin configuration which
    it reads.
    """
    
    DASHBOARD_SOURCES_LOCATION = 'src'
    
    def __init__(self, parent, xml_register_dir):
        """! The constructor."""
        
        self._context = parent.getContext()
        
        #Check dir 
        if not os.path.isfile(xml_register_dir):
            raise CobotGuiException('Dashboard register file "%s" in package "cobot_gui" not found'
                                     %(xml_register_dir))
        
        #Parse xml file
        try:
            self._dashboard_register = ElementTree.parse(xml_register_dir)
        except Exception as e:
            raise CobotGuiException(str(e))
        
    def getPkgByName(self, name):
        
        root = self._dashboard_register.getroot()
        
        #Find and read node label
        dashboard_desc = root.find('./dashboard[@label="%s"]'%name)
        
        if dashboard_desc is None:
            raise CobotGuiException('Cannot foud package from dashboard named "%s"'%name)
        
        return dashboard_desc.attrib['package']
    
    def getInstance(self, dashboard_name, dashboard_node=None):
        """! Load Python package.
        @param descriptor: package descriptor xml node.
        @type descriptor: Element.
        
        @param uuid: dashboard id.
        @type uuid: string.
        
        @return dashboard_instance: dashboard instance.
        @type dashboard_instance: Dashboard.
        """
        
        dashboard_pkg_name = self.getPkgByName(dashboard_name)
        
        dashboard_dir = get_pkg_dir(dashboard_pkg_name)
        
        dashboard_descriptor_file = os.path.join(dashboard_dir,"dashboard_descriptor.xml")
        
        if not os.path.isfile(dashboard_descriptor_file):
            self._context.getLogger().err('Cannot found dashboard_descriptor.xml into plugin %s'%dashboard_name)
            return None
        
        dashboard_descriptor_root = ElementTree.parse(dashboard_descriptor_file).getroot()
        
        dashboard_import = dashboard_descriptor_root.find('import')
         
        dashboard_module_path = dashboard_import.attrib['module']
        dashboard_class_name = dashboard_import.attrib['class']
        
        sys.path.append(os.path.join(dashboard_dir,self.DASHBOARD_SOURCES_LOCATION))
        
        dashboard_class_ref = None
        #Import package module
        #Raise exception
        try:
            module = __builtin__.__import__(dashboard_module_path,
                                            fromlist=[dashboard_class_name],
                                            level=0)
            
        except Exception as ex:
            self._context.getLogger().err("Cannot import plugin '%s' !\n%s"%(dashboard_name, str(ex)))
            return None
        
        #Get referance to plugin class
        dashboard_class_ref = getattr(module, dashboard_class_name)
            
        if dashboard_class_ref is None:
            self._context.getLogger().err("Cannot found plugin class '%s' !"%dashboard_class_name)
            return None
        
        dashboard_instance = dashboard_class_ref(self._context)
        
        dashboard_params = DashboardProvider.getParameters(dashboard_descriptor_root, dashboard_node)
        
        dashboard_instance.setup(dashboard_descriptor_root, dashboard_params)
        
        return dashboard_instance
    
    @staticmethod
    def getParameters(dashboard_descriptor, dashboard_node):
        
        parameters = Parameters()
        
        # Try to provide plugin parameters in dashboard_descriptor.xml
        descriptor_params = dashboard_descriptor.find("setup/parameters")
        
        # Add parameters, if parameters found in dashboard_descriptor.xml
        if descriptor_params is not None:
            for param in descriptor_params:
                # Append parameters
                parameters.putParam(param.attrib['name'], param.attrib['value'])
                
        if dashboard_node is not None:
            # Check if parameters remapped in cobot_gui config launch
            if dashboard_node.find("param") is not None:
                
                for param in dashboard_node.iter('param'):
                    # Update or append parameters
                    parameters.putParam(param.attrib['name'], param.attrib['value'])
        
        return parameters
        
##Unittest
if __name__ == "__main__":
    
    from python_qt_binding.QtGui import *
    from python_qt_binding.QtCore import *
    from cobot_gui.context import Context
    
    rospy.init_node('dashboard_privider_test')
    
    a = QApplication(sys.argv)
    utt_appli = QMainWindow()

    context = Context(utt_appli)
    
    provider = DashboardProvider(context, "/opt/ros/indigo/workspace/src/gui/dashboard/dashboard_register.xml")
    
    dashboard = provider.getDashboardInstance("IIWA")
    
    utt_appli.setCentralWidget(dashboard)
    
    plugin.onStart()
    
    utt_appli.show()
    a.exec_()
    
    

#End of file

