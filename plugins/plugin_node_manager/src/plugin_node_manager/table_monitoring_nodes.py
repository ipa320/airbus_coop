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
import re
import subprocess
import rosnode

from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from python_qt_binding import loadUi

from custom_rosnode import rosnode_ping_all, rosnode_cleanup
from node_item import NodeItem

from cobot_gui import Alarm

from plugin_node_manager.res import R

class ThreadNodePingAll(QThread):
    def __init__(self):
        QThread.__init__(self)
        
    def run(self):
        
        while not rospy.is_shutdown():
            
            try:
                alive_nodes, dead_nodes = rosnode_ping_all()
                self.emit(SIGNAL("pingStates"), alive_nodes, dead_nodes)
            except:
                pass
            
            time.sleep(2.0)

class TableMonitoringNodes:
    
    def __init__(self, parent):
        
        self._parent = parent
        
        QObject.connect(self._parent._but_cleanup,
                        SIGNAL('clicked()'),self.onCleanup)
        
        self._datamodel = QStandardItemModel(0, 5)
        
        
        self._parent._table_monitor.setModel(self._datamodel)
        self._parent._table_monitor.verticalHeader().setVisible(False)
        
        self._master = rosnode.rosgraph.Master(rosnode.ID)
        self._table_dict = {} #{node name : NodeItem()}
        self._qtable_index = 0 #Row index
        self._deaded_nodes_alarm_register = []
        
        self._init_node_table()
        
        import threading
        self._mutex = threading.Lock()
        
        self.thread_rosnode_ping_all = ThreadNodePingAll()
        QObject.connect(self.thread_rosnode_ping_all,
                        SIGNAL('pingStates'), self._refresh_node_table)
        
    def onStart(self):
        self.onCleanup()
        self.thread_rosnode_ping_all.start()
        
    def onCleanup(self):
        
        rosnode_cleanup()
        
        with self._mutex:
            for i in range(self._datamodel.rowCount()):
                self._datamodel.removeRow(i)
            self._table_dict = {}
            self._init_node_table()
            self._deaded_nodes_alarm_register = []
    
    def _init_node_table(self):
        
        nodes = rosnode._sub_rosnode_listnodes()
        nodelist = nodes.split('\n')
        
        for i in range(len(nodelist)):
            uri = rosnode.get_api_uri(self._master, nodelist[i])
            uri = uri.replace('/','')
            uri = '/'+uri.split(':')[1]
            self._add_node(uri, nodelist[i], i)
            
        self._qtable_index = len(nodelist)
        
        self._parent._table_monitor.resizeColumnsToContents()
        self._parent._table_monitor.resizeRowsToContents()
        
    def _add_node(self, uri, node, index):
        
        new_node = NodeItem(uri,node)
        
        self._datamodel.setItem(index, 0, QStandardItem())
        
        qindex = self._datamodel.index(index, 0, QModelIndex())
        self._parent._table_monitor.setIndexWidget(qindex, new_node.uri)
        
        qindex = self._datamodel.index(index, 1, QModelIndex())
        self._parent._table_monitor.setIndexWidget(qindex, new_node.name)
        
        qindex = self._datamodel.index(index, 2, QModelIndex())
        self._parent._table_monitor.setIndexWidget(qindex, new_node.status)
        
        qindex = self._datamodel.index(index, 3, QModelIndex())
        self._parent._table_monitor.setIndexWidget(qindex, new_node.ping)
        
        qindex = self._datamodel.index(index, 4, QModelIndex())
        self._parent._table_monitor.setIndexWidget(qindex, new_node.button_start_stop_widget)
        
        self._table_dict.update({node:new_node})
        
    def _refresh_node_table(self, alive_nodes, dead_nodes):
        
        with self._mutex:
            
            self._nb_running = len(alive_nodes)
            
            if alive_nodes.keys() != self._table_dict.keys():
                self._update_node_table(alive_nodes)
            
            for node, ping_time in alive_nodes.items():
                if node in self._table_dict.keys():
                    self._table_dict[node].refresh(NodeItem.RUNNING, ping_time)
            
            for node in dead_nodes:
                
                if node not in self._deaded_nodes_alarm_register:
                    self._deaded_nodes_alarm_register.append(node)
                    self._parent.sendAlarm(Alarm.WARNING, "The node %s is dead !"%node)
                    
                if node in self._table_dict.keys():
                    self._table_dict[node].refresh(NodeItem.ABARTED)
            
    def _update_node_table(self, alive_nodes):
        
        for node in alive_nodes.keys():
            if node not in self._table_dict.keys():
                uri = rosnode.get_api_uri(self._master, node)
                uri = uri.replace('/','')
                uri = '/'+uri.split(':')[1]
                self._add_node(uri, node, self._qtable_index)
                self._qtable_index += 1
        
        for node in self._table_dict.keys():
            if node not in alive_nodes.keys():
                
                self._table_dict[node].refresh(NodeItem.SHUTDOWN)
                
                if node not in self._deaded_nodes_alarm_register:
                    self._deaded_nodes_alarm_register.append(node)
                    self._parent.sendAlarm(Alarm.WARNING, "The node %s is dead !"%node)
        
        self._parent._table_monitor.resizeColumnsToContents()
        self._parent._table_monitor.resizeRowsToContents()
        
    def translate(self, lng):
        
        self._parent.monitor_header_label.setText(R.values.strings.nodes_manager_title(lng))
        self._parent._but_cleanup.setText(R.values.strings.cleanup(lng))
        
        header_label = [R.values.strings.machine(lng),
                        R.values.strings.node(lng),
                        R.values.strings.status(lng),
                        R.values.strings.ping(lng),
                        R.values.strings.start_stop(lng)]
        
        for i in range(len(header_label)):
            self._datamodel.setHeaderData(i, Qt.Horizontal, header_label[i])
    
#End of file
