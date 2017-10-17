from roslib.packages import get_pkg_dir
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from airbus_cobot_gui.context import Context
from plugin_provider import PluginProvider
from plugin import Plugin
from xml.etree import ElementTree as ET

def getStandAloneInstance(pkg_name, plugin_class, lng="en"):
    
    plugin_instance = plugin_class(Context(QMainWindow()))
    
    plugin_descriptor = ET.parse("%s/plugin_descriptor.xml"%get_pkg_dir(pkg_name))
    
    plugin_params = PluginProvider.getParameters(plugin_descriptor.getroot(), None)
    
    plugin_instance.tryToCreate(plugin_params)
    
    plugin_instance.tryToResume()
    
    plugin_instance.onTranslate(lng)
    
    return plugin_instance
