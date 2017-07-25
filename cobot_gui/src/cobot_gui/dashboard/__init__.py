################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : plugin.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

from dashboard_provider import DashboardProvider
from dashboard import Dashboard
from wrapper_dashboard import DashboardPopup

from roslib.packages import get_pkg_dir
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from cobot_gui.context import Context
from xml.etree import ElementTree as ET

def getStandAloneInstance(pkg_name, dashboard_class, lng="en"):
    
    dashboard_instance = dashboard_class(Context(QMainWindow()))
    
    dashboard_descriptor = ET.parse("%s/dashboard_descriptor.xml"%get_pkg_dir(pkg_name))
    
    dashboard_params = DashboardProvider.getParameters(dashboard_descriptor.getroot(), None)
    
    dashboard_instance.setup(dashboard_descriptor, dashboard_params)
    
    dashboard_instance.onTranslate(lng)
    
    return dashboard_instance
