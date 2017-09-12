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

import os
import datetime

import rospy

from std_msgs.msg import String, Empty, Bool, Int8
from rosgraph_msgs.msg import Log

from xml.etree import ElementTree as ET

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from airbus_pyqt_extend import QtAgiCore

from airbus_cobot_gui import plugin, ControlMode, EmergencyStopState

from scxml import SCXMLState
from airbus_ssm_core import ssm_main
from airbus_ssm_core.srv import SSM_init


from ast import literal_eval
from functools import partial
from PyQt4.Qt import QByteArray


SERVER_NAME = '/ssm'
#####################################

from res import R

class SSMRunnable(QThread):
    
    def __init__(self, parent):
        QThread.__init__(self, parent)
        self.SSM_Main = ssm_main.ssmMain()
        
    def run(self):
        while(rospy.is_shutdown == False):
            rospy.spinOnce()
            rospy.sleep(0.1)
        

class SSMIntrospection(plugin.Plugin):
    
    trigger_status = pyqtSignal()
    trigger_treeview = pyqtSignal()
    trigger_log = pyqtSignal()
    
    def __init__(self, context):
        plugin.Plugin.__init__(self, context)
        
    def onCreate(self, param):
        
        loadUi(R.layouts.mainwindow, self)
        
        self._scxml_model = QStandardItemModel()
        self._scxml_model.setHorizontalHeaderLabels([R.values.strings.scxml_header()])
        self.scxml_treeview.setModel(self._scxml_model)
        self.scxml_treeview.setIconSize(QSize(24,24))       
        
        ##SET UP BUTTON
        self.open_button.setIcon(QIcon(R.images.ic_folder))
        self.open_button.setIconSize(QSize(50,50))
        self.open_button.setEnabled(False)
        self.connect(self.open_button, SIGNAL('clicked()'), self.openSCXMLFile)
        
        self.preempt_button.setIcon(QIcon(R.images.ic_preempt))
        self.preempt_button.setIconSize(QSize(100,100))
        self.preempt_button.setEnabled(False)
        self.connect(self.preempt_button, SIGNAL('clicked()'), self._preempt_ssm)
        
        self.start_button.setIcon(QIcon(R.images.ic_start))
        self.start_button.setIconSize(QSize(50,50))
        self.start_button.setEnabled(False)
        self.connect(self.start_button, SIGNAL('clicked()'), self._start_button_clicked)
        
        self.pause_button.setIcon(QIcon(R.images.ic_pause))
        self.pause_button.setIconSize(QSize(50,50))
        self.pause_button.setEnabled(False)
        self.connect(self.pause_button, SIGNAL('clicked()'), self._pause_button_clicked)

        self.rearm_button.setIcon(QIcon(R.images.ic_rearm))
        self.rearm_button.setIconSize(QSize(80,80))
        self.rearm_button.setEnabled(False)
        self.connect(self.rearm_button, SIGNAL('clicked()'), self._rearm_button_clicked)
        
         ##Setup Thread
        
        self._ssm_runnable = SSMRunnable(self)
        self._ssm_runnable.start()
        
        ##Setup Publisher / Subscriber
        self._server_name = rospy.get_param('ssm_server_name', '/ssm')

        self._ssm_tree_view_sub = rospy.Subscriber(self._server_name+'/ssm_status',
                                                    String,
                                                    self._ssm_tree_view_cb)

        self._ssm_ready_sub = rospy.Subscriber(self._server_name + '/status',
                                               Int8,
                                               self._ssm_status_cb, queue_size=1)
        
        self._log_sub = rospy.Subscriber('/rosout', 
                                         Log, 
                                         self._log_cb)
        
        self._preempt_pub = rospy.Publisher(self._server_name+'/preempt', Empty, queue_size=1)
        
        self._start_pub = rospy.Publisher(self._server_name+'/start', Empty, queue_size=1)
        
        self._pause_pub = rospy.Publisher(self._server_name+'/pause', Bool, queue_size=1)
        
        self._request_tree_view_pub = rospy.Publisher(self._server_name+'/status_request', Empty, queue_size=1)
        
        self.trigger_status.connect(self.updateStatus)
        self.trigger_treeview.connect(self.updateTreeView)
        self.trigger_log.connect(self.updateLog)

        self._ssm_status = 0
        self.ssm_status.setPixmap(R.getPixmapById("ic_ssm_pending").scaled(50,50))
        
        self._tree_view_dict = None
        #self._log = ""

        self._tree_view = False
        self._ssm_loaded = False
        self._ssm_paused = False
        self._auto_reload = False
        
           
    def clearAllStates(self):
        self._tree_view = False
        self._scxml_model.clear()
        
    ###STATUS DEFINITION
    def _not_loaded(self):
        self._clear_tree_view()
        self.ssm_status.setPixmap(R.getPixmapById("ic_ssm_pending").scaled(50,50))
        self.status_label.setText("NOT LOADED ")
        self.start_button.setEnabled(False)
        self.pause_button.setEnabled(False)
        self.open_button.setEnabled(True)
        self.rearm_button.setEnabled(False)
        self.preempt_button.setEnabled(False)
        
    def _ready_state(self):
        ready_gif = QMovie(R.images.ic_ready)
        ready_gif.setCacheMode(QMovie.CacheAll)
        ready_gif.setScaledSize(QSize(50,50))
        ready_gif.setSpeed(100)
        self.ssm_status.setMovie(ready_gif)
        ready_gif.start()
        self.status_label.setText("READY ")
        self.start_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.open_button.setEnabled(True)
        self.rearm_button.setEnabled(False)
        self.preempt_button.setEnabled(False)
        self.rearm_button.setStyleSheet("background-color: transparent")
    
    def _running_state(self):
        self.ssm_status.setPixmap(R.getPixmapById("ic_ssm_ok").scaled(50,50))
        self.status_label.setText("RUNNING ")
        self.start_button.setEnabled(False)
        self.pause_button.setEnabled(True)
        self.open_button.setEnabled(False)
        self.rearm_button.setEnabled(False)
        self.preempt_button.setEnabled(True)
    
    def _pause_state(self):
        self.ssm_status.setPixmap(R.getPixmapById("ic_ssm_pause").scaled(50,50))
        self.status_label.setText("PAUSE ")
        self.start_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.open_button.setEnabled(False)
        self.rearm_button.setEnabled(False)
        self.preempt_button.setEnabled(True)
        
    def _preempt_state(self):
        self.ssm_status.setPixmap(R.getPixmapById("ic_ssm_error").scaled(50,50))
        self.status_label.setText("ERROR ")
        self.start_button.setEnabled(False)
        self.pause_button.setEnabled(False)
        self.open_button.setEnabled(False)
        self.rearm_button.setEnabled(True)
        self.rearm_button.setStyleSheet("background-color: red")
        self.preempt_button.setEnabled(False)
        
    def _finish_state(self):
        self.ssm_status.setPixmap(R.getPixmapById("ic_ssm_ok").scaled(50,50))
        self.status_label.setText("FINISHED ")
        self.start_button.setEnabled(False)
        self.pause_button.setEnabled(False)
        self.open_button.setEnabled(True)
        self.rearm_button.setEnabled(False)
        self.preempt_button.setEnabled(False)
        if(self.auto_reload.isChecked()):
            self._load_SSM()
        else:
            self._not_loaded()
        
    def updateLog(self):
        self.log_txt.setText(self._log)
        
    
    def updateStatus(self):
            status = self._ssm_status 
            if status == 0:
                self._not_loaded()
            elif status == 1: ##Ready
                self._ready_state()
            elif status == 2: ##Running
                self._running_state()
            elif status == -1: ##Pause
                self._pause_state()
            elif status == -2: ##Error / Preempt
                self._preempt_state()
            elif status == -10: ##Error / Preempt
                self._not_loaded()
            elif status == 10: ##Finish
                self._finish_state()
          
    def updateTreeView(self):

        if (self._tree_view == False):
            self._construct_tree_view(self._tree_view_dict)
            self._tree_view = True
        else:
            self._update_tree_view(self._tree_view_dict,self._scxml_model.item(0))
            
    def _log_cb(self, log):
        if(log.name == rospy.get_name()):
            strtime = datetime.datetime.now().strftime("[%Y/%m/%d-%H:%M:%S] ")
            self._log = strtime+str(log.msg)
            self.trigger_log.emit()
    
    def _ssm_tree_view_cb(self, msg):
        self._tree_view_dict = literal_eval(str(msg.data))
        self.trigger_treeview.emit()
            
    def _ssm_status_cb(self, msg):
        self._ssm_status = msg.data
        self.trigger_status.emit()
        
    
    def openSCXMLFile(self): 
        
        self._ssm_status = 0
        self.updateStatus()
              
        fdial = QFileDialog()
        try:
            rospy.get_param("/ssm_node/scxml_file")
            default_f = rospy.get_param("/ssm_node/scxml_file")
            
        except KeyError:
            default_f = QtAgiCore.get_pkg_dir_from_prefix("${airbus_ssm_core}")
            
        scxmlfile = fdial.getOpenFileName(self, "Open SCXML",'',"SCXML (*.scxml)", None, QFileDialog.DontUseNativeDialog)
        if scxmlfile[0] == "":
            return

        self._scxml_model.setHorizontalHeaderLabels([scxmlfile[0]])
        rospy.set_param('/ssm_node/scxml_file',scxmlfile[0])
        self._load_SSM()
        
        
            
    def _load_SSM(self):
        try:
            result = self._wait_ssm_isready()
            if(result == True):
                self._ssm_loaded = True
                self._wait_tree_view()
            else:
                self._not_loaded()             
        except KeyError as e:
             self._not_loaded()
    
    def _wait_ssm_isready(self):
        rospy.wait_for_service(self._server_name+'/srv/init',10)
        try:
            _init_srv = rospy.ServiceProxy(self._server_name+'/srv/init',SSM_init)
            call_ = String()
            call_.data = rospy.get_param("/ssm_node/scxml_file")
            resp = _init_srv(call_)
            return resp.result.data
        except rospy.ServiceException, e:
            rospy.logerr('Service call failed : %s'%e)
            raise rospy.ServiceException(e)

    def _wait_tree_view(self):
        while(rospy.is_shutdown() == False and self._tree_view_dict == None ):
            self._request_tree_view_pub.publish()
            rospy.sleep(0.1)
                
    def _start_button_clicked(self):
        if(self._ssm_paused == True):
            self._ssm_paused = False
            self._pause_pub.publish(False) 
        else :
            self._start_pub.publish() 

    def _pause_button_clicked(self):
        if(self._ssm_paused == False):
            self._ssm_paused = True
            self._pause_pub.publish(True)
            
    def _rearm_button_clicked(self):
        self._load_SSM()
        
    def _preempt_ssm(self):
        self._preempt_pub.publish()
        self._ssm_paused = False
        
    def _construct_tree_view(self, tree_view):

        graph = SCXMLState("ROOT",SCXMLState.PARENT_STATE)
        #FIND ROOT
        for state, value in tree_view.iteritems():
            if state=="ROOT":
                self._add_states_into_treeview(state, value, graph, tree_view)
        
        self._scxml_model.appendRow(graph)
        self.scxml_treeview.expandAll()
    
    def _add_states_into_treeview(self, state, value, graph, tree_view):
        
        for state_, value_ in value.iteritems():
            child = SCXMLState(state_, SCXMLState.SIMPLE_STATE)
            graph.appendRow(child)
            for state__, value__ in tree_view.iteritems():
                if(state_ == state__):
                    child.setType(SCXMLState.PARENT_STATE)
                    self._add_states_into_treeview(state__, value__, child, tree_view)    
        
    def _update_tree_view(self, tree_view, item):
        if(item.type() == SCXMLState.PARENT_STATE):
            for state, value in tree_view.iteritems():
                if(item.get_name() == state):
                    for i_item in range(item.rowCount()):
                        for state_, value_ in value.iteritems():
                            if(item.child(i_item).get_name() == state_):    
                                item.child(i_item).setStatus(value_)
                        self._update_tree_view(tree_view, item.child(i_item))
                        
    def _clear_tree_view(self):
        self._tree_view_dict = None
        self._tree_view = False
        self._scxml_model.clear() 
        
    def onPause(self):
        pass
    
    def onResume(self):
        if(self._ssm_loaded == False):
             self._load_SSM()
        
        
    def onControlModeChanged(self, mode):
        
        if mode == ControlMode.AUTOMATIC:
            self.setEnabled(False)
        else:
            self.setEnabled(True)
        
    def onUserChanged(self, user_info):
        pass
    
    def onTranslate(self, lng):
        pass
    
    def onEmergencyStop(self, state):
        self.onPause()
        
    def onDestroy(self):
        self._preempt_ssm()
        
if __name__ == "__main__":
    
    import sys
    import signal
    
    rospy.init_node("airbus_ssm_plugin_node")
    
    a = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    window = plugin.getStandAloneInstance("airbus_ssm_plugin", SSMIntrospection, "en")
    window.setWindowTitle("ssmIntrospection")
    window.show()
    
    sys.exit(a.exec_())


    
    
