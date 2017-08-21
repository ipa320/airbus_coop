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
import time
import os

from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from launch_item import LaunchItem

class TableLaunchNodes:
    
    def __init__(self, gui):
        
        self._gui = gui
        
        self._datamodel = QStandardItemModel(0, 3)
        
        self._gui._table_launch.setModel(self._datamodel)
        self._gui._table_launch.verticalHeader().setVisible(False)
        
        node_launchers_path = '/'.join([get_pkg_dir('node_launchers'),
                                        'launch'])
        
        self._launch_list = []
        
#         self._gui._table_launch.horizontalHeader().setResizeMode(QHeaderView.Stretch)
        
        i=0
        for launch_file in os.listdir(node_launchers_path):
            self._add_launcher(launch_file,'/cobotgui-dev', i)
            i+=1
        
        self._gui._table_launch.resizeColumnsToContents()
        self._gui._table_launch.resizeRowsToContents()
    
    def _add_launcher(self, launch_file, machine_name, index):
        
        new_launch = LaunchItem(launch_file, machine_name)
        
        self._datamodel.setItem(index, 0, QStandardItem())
        
        qindex = self._datamodel.index(index, 0, QModelIndex())
        self._gui._table_launch.setIndexWidget(qindex, new_launch.launch_name)
        
        qindex = self._datamodel.index(index, 1, QModelIndex())
        self._gui._table_launch.setIndexWidget(qindex, new_launch.combo_machines)
        
        qindex = self._datamodel.index(index, 2, QModelIndex())
        self._gui._table_launch.setIndexWidget(qindex, new_launch.button_launch_widget)
        
        self._launch_list.append(new_launch)
        
    def translate(self):
        
        self._gui.launch_header_label.setText(trUtf8('Nodes launch'))
        
        header_label = [trUtf8('Name'), trUtf8('Default machine'), trUtf8('Launch')]
        
        for i in range(len(header_label)):
            self._datamodel.setHeaderData(i, Qt.Horizontal, header_label[i])
    
#End of file
