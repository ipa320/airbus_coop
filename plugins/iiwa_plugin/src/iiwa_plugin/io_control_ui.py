#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : cartesian_move.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

import rospy
import os

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from arm_msgs.msg import IOStates, Digital

from robot_axis_widget import RobotAxisWidget

from pyqt_agi_extend.QtAgiCore import QAgiSubscriber
from pyqt_agi_extend.QtAgiGui import QAgiSilderButton

from iiwa_driver import IIWAIOMediaFlange

from iiwa_plugin.res import R

class IOControlUi(QWidget):
    
    def __init__(self):
        QWidget.__init__(self)
        
        loadUi(R.layouts.io_control, self)
        
        self.setObjectName("ioControl")
        
        self._ioMediaFlange = IIWAIOMediaFlange()
        
        self._io_states = IOStates()
        self._io_states.digital_input_states = 8*[Digital()]
        self._io_states.digital_output_states = 4*[Digital()]
        
        self._io_states_comm = QAgiSubscriber(self,
                                              '/iiwa/io_states',
                                              IOStates,
                                              self._slot_update_current_io_states,
                                              max_rate = 10)
        
        self._output1 = QAgiSilderButton(initial_state=False, on_label='TRUE', off_label='FALSE')
        self._output1.setBackgroundStyle(R.values.styles.io_background)
        self._output1.setOnStyle(R.values.styles.io_on)
        self._output1.setOffStyle(R.values.styles.io_off)
        self.connect(self._output1, SIGNAL('statusChanged'), self._slot_output1_trigg)
        
        self._output2 = QAgiSilderButton(initial_state=False, on_label='TRUE', off_label='FALSE')
        self._output2.setBackgroundStyle(R.values.styles.io_background)
        self._output2.setOnStyle(R.values.styles.io_on)
        self._output2.setOffStyle(R.values.styles.io_off)
        self.connect(self._output2, SIGNAL('statusChanged'), self._slot_output2_trigg)
        
        self._output3 = QAgiSilderButton(initial_state=False, on_label='TRUE', off_label='FALSE')
        self._output3.setBackgroundStyle(R.values.styles.io_background)
        self._output3.setOnStyle(R.values.styles.io_on)
        self._output3.setOffStyle(R.values.styles.io_off)
        self.connect(self._output3, SIGNAL('statusChanged'), self._slot_output3_trigg)
        
        self._output4 = QAgiSilderButton(initial_state=False, on_label='TRUE', off_label='FALSE')
        self._output4.setBackgroundStyle(R.values.styles.io_background)
        self._output4.setOnStyle(R.values.styles.io_on)
        self._output4.setOffStyle(R.values.styles.io_off)
        self.connect(self._output4, SIGNAL('statusChanged'), self._slot_output4_trigg)
        
        self.output1_layout.addWidget(self._output1)
        self.output2_layout.addWidget(self._output2)
        self.output3_layout.addWidget(self._output3)
        self.output4_layout.addWidget(self._output4)
        
        for i in range(0,8):
            getattr(self, "digital_input%i"%(i+1)).setStyleSheet(R.values.styles.io_off)
        
    def _slot_output1_trigg(self, state):
        self._ioMediaFlange.setDigitalOutputState(1, state, IIWAIOMediaFlange.ASYNC)
        
    def _slot_output2_trigg(self, state):
        self._ioMediaFlange.setDigitalOutputState(2, state, IIWAIOMediaFlange.ASYNC)
        
    def _slot_output3_trigg(self, state):
        self._ioMediaFlange.setDigitalOutputState(3, state, IIWAIOMediaFlange.ASYNC)
        
    def _slot_output4_trigg(self, state):
        self._ioMediaFlange.setDigitalOutputState(4, state, IIWAIOMediaFlange.ASYNC)
    
    def _slot_update_current_io_states(self, io):
        
        for i in range(0, len(self._io_states.digital_input_states)):
            
            input_ui = getattr(self, "digital_input%i"%(i+1))
            
            if io.digital_input_states[i].state != self._io_states.digital_input_states[i].state :
                
                if io.digital_input_states[i].state is True:
                    input_ui.setStyleSheet(R.values.styles.io_on)
                    input_ui.setText("TRUE")
                else:
                    input_ui.setStyleSheet(R.values.styles.io_off)
                    input_ui.setText("FALSE")
                    
        for i in range(0, len(self._io_states.digital_output_states)):
            
            output_ui = getattr(self, "_output%i"%(i+1))
            
            if io.digital_output_states[i].state != self._io_states.digital_output_states[i].state :
                
                if io.digital_output_states[i].state is True:
                    output_ui.setState(state=True, anonymous=True)
                else:
                    output_ui.setState(state=False, anonymous=True)
        
        self._io_states = io
        
    def _publish_digital_output_command(self):
        pass
        
    def subscribe(self):
        self._ioMediaFlange.waitForRobotConnection(rospy.Duration())
        self._io_states_comm.subscribe()
        
    def unregister(self):
        self._ioMediaFlange.shutdown()
        self._io_states_comm.unregister()
        
    def resizeEvent(self, event):
        pass
        
