#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : launcher_menu.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

## @package: autorun
##
## @version 1.0
## @author  Matignon Martin
## @date    Last modified 21/02/2014

import sys
import os

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from cobot_gui.menu_launcher import MenuLauncher


if __name__ == "__main__":
    
    app = QApplication(sys.argv)
    
    window = QMainWindow()
    window.setWindowTitle("Cobot_gui startup")
    
    window.setCentralWidget(MenuLauncher())
    
    window.show()
    
    app.exec_()
    
#@endcond

