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


from rosgraph_msgs.msg import Log
import rospkg
import rospy

from python_qt_binding.QtCore import QMutex, QMutexLocker, QTimer
from python_qt_binding.QtGui import QWidget, QGridLayout

from rqt_console.console_settings_dialog import ConsoleSettingsDialog
from rqt_console.console_widget import ConsoleWidget
from rqt_console.message import Message
from rqt_console.message_data_model import MessageDataModel
from rqt_console.message_proxy_model import MessageProxyModel

from cobot_gui import Plugin, ControlMode

class PluginLogManager(Plugin):
    """
    rqt_console plugin's main class. Handles communication with ros_gui and contains
    callbacks to handle incoming message
    """
    def __init__(self, context):
        Plugin.__init__(self, context)
        
    def onCreate(self, param):
        
        layout = QGridLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)
        
        self._rospack = rospkg.RosPack()
        
        self._model = MessageDataModel()
        self._proxy_model = MessageProxyModel()
        self._proxy_model.setSourceModel(self._model)
        
        self.__widget = ConsoleWidget(self._proxy_model, self._rospack)
        
        layout.addWidget(self.__widget, 0, 0, 0, 0)
        
        # queue to store incoming data which get flushed periodically to the model
        # required since QSortProxyModel can not handle a high insert rate
        self._message_queue = []
        self._mutex = QMutex()
        self._timer = QTimer()
        self._timer.timeout.connect(self.insert_messages)
        self._timer.start(100)
         
        self._subscriber = None
        self._topic = '/rosout_agg'
        self._subscribe(self._topic)
        
    def queue_message(self, log_msg):
        """
        Callback for adding an incomming message to the queue.
        """
        if not self.__widget._paused:
            msg = PluginLogManager.convert_rosgraph_log_message(log_msg)
            with QMutexLocker(self._mutex):
                self._message_queue.append(msg)
 
    @staticmethod
    def convert_rosgraph_log_message(log_msg):
        msg = Message()
        msg.set_stamp_format('hh:mm:ss.ZZZ (yyyy-MM-dd)')
        msg.message = log_msg.msg
        msg.severity = log_msg.level
        msg.node = log_msg.name
        msg.stamp = (log_msg.header.stamp.secs, log_msg.header.stamp.nsecs)
        msg.topics = sorted(log_msg.topics)
        msg.location = log_msg.file + ':' + log_msg.function + ':' + str(log_msg.line)
        return msg
     
    def insert_messages(self):
        """
        Callback for flushing incoming Log messages from the queue to the model.
        """
        with QMutexLocker(self._mutex):
            msgs = self._message_queue
            self._message_queue = []
        if msgs:
            self._model.insert_rows(msgs)
             
    def shutdown_plugin(self):
        self._subscriber.unregister()
        self._timer.stop()
        self.__widget.cleanup_browsers_on_close()
         
    def save_settings(self, plugin_settings, instance_settings):
        self.__widget.save_settings(plugin_settings, instance_settings)
         
    def restore_settings(self, plugin_settings, instance_settings):
        self.__widget.restore_settings(plugin_settings, instance_settings)
         
    def trigger_configuration(self):
        topics = [t for t in rospy.get_published_topics() if t[1] == 'rosgraph_msgs/Log']
        topics.sort(key=lambda tup: tup[0])
        dialog = ConsoleSettingsDialog(topics, self._rospack)
        (topic, message_limit) = dialog.query(self._topic, self._model.get_message_limit())
        if topic != self._topic:
            self._subscribe(topic)
        if message_limit != self._model.get_message_limit():
            self._model.set_message_limit(message_limit)
     
    def _subscribe(self, topic):
        if self._subscriber:
            self._subscriber.unregister()
        self._subscriber = rospy.Subscriber(topic, Log, self.queue_message)
        self._currenttopic = topic
        
    def onStart(self):
        pass
        
    def onPause(self):
        pass
    
    def onResume(self):
        pass
        
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
        pass
        
    def onDestroy(self):
        self.shutdown_plugin()
