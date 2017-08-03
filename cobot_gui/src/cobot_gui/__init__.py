#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : cobot_gui.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

# from cobot_gui.pkg_dir import cobot_gui_dir, resources_dir

# from cobot_gui.translator.libtraduc import tr, trUtf8

from cobot_gui.util import CobotGuiException 

from cobot_gui.account import User, Privilege
from cobot_gui.account import Privilege as UserPrivilege
from cobot_gui.control_mode import ControlMode
from cobot_gui.emergency import EmergencyStopState

from cobot_gui.alarm import Alarm

from cobot_gui import plugin
from cobot_gui.plugin.plugin import Plugin

from cobot_gui import dashboard
from cobot_gui.dashboard import DashboardPopup, Dashboard

# from cobot_gui.python_qt_extend.qsilderbutton import QSilderButton
# from cobot_gui.python_qt_extend.qpopup import QPopup
# from cobot_gui.python_qt_extend.message_box import *
