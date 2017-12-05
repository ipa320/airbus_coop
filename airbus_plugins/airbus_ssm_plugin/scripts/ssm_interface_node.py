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
import sys

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *

from airbus_ssm_plugin.ssm_viewer import SSMViewer

from airbus_cobot_gui import plugin

if __name__ == "__main__":
    
    import sys
    import signal
    
    rospy.init_node("airbus_ssm_plugin_node")
    
    a = QApplication(sys.argv)
    
    window = plugin.getStandAloneInstance("airbus_ssm_plugin", SSMViewer, "en")
    window.setWindowTitle("ssmIntrospection")
    window.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)

    sys.exit(a.exec_())
    
#End of file
