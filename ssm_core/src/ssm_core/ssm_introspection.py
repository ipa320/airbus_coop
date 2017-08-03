#!/usr/bin/env python

from std_msgs.msg import String, Empty

import threading
import smach
import rospy
from ast import literal_eval



class ssmIntrospection():
    def __init__(self, state_machine):
        """Traverse the smach tree starting at root, and construct introspection
        proxies for getting and setting debug state."""

        self._state_machine = state_machine
        self._tree_view = {}
        self._server_name = rospy.get_param('ssm_server_name', '/ssm')
        self._update_pub = rospy.Publisher(self._server_name + "/ssm_status", String, queue_size = 1)
        self._request_update = rospy.Subscriber(self._server_name + "/status_request", Empty, self._status_request_cb, queue_size = 1)

    def start(self):
        # Construct proxies
        self._state_machine._create_tree_view()
        self.construct(self._state_machine,"ROOT")
    
    def stop(self):
        self._request_update.unregister()
        self._tree_view = {}
        

    def construct(self, state, _key):
        """Recursively construct tree view."""
        self._tree_view[_key] = state.get_tree_view()
        state.register_tree_view_cb(self._update_tree_view_cb)
        # Get a list of children that are also containers
        for (label, child) in state.get_children().items():
            # If this is also a container, recurse into it
            if isinstance(child, smach.container.Container):
                child._create_tree_view()
                self.construct(child, label)
                
    def _update_tree_view_cb(self, *args, **kwargs):
        self._update_pub.publish(str(self._tree_view))
        
    def _status_request_cb(self, msg):
        self._update_pub.publish(str(self._tree_view))
        
        