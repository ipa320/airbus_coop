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


import threading
import datetime
import traceback

from std_msgs.msg import Empty, Bool

import rospy
import smach

__all__ = ['StateMachine']



def StrTimeStamped():
    return datetime.datetime.now().strftime("%Y_%m_%d_%H:%M:%S.%f   - ")

### State Machine class
class ssmStateMachine(smach.StateMachine):
    """StateMachine
    
    This is a finite state machine smach container. Note that though this is
    a state machine, it also implements the L{smach.State}
    interface, so these can be composed hierarchically, if such a pattern is
    desired.

    States are added to the state machine as 3-tuple specifications:
     - label
     - state instance
     - transitions

    The label is a string, the state instance is any class that implements the
    L{smach.State} interface, and transitions is a dictionary mapping strings onto
    strings which represent the transitions out of this new state. Transit_preempt_pauseions
    can take one of three forms:
     - OUTCOME -> STATE_LABEL
     - OUTCOME -> None (or unspecified)
     - OUTCOME -> SM_OUTCOME
    """
    def __init__(self, outcomes,input_keys=[], output_keys=[]):
        
        input_keys.append("logfile")
        output_keys.append("logfile")
        smach.StateMachine.__init__(self,outcomes, input_keys, output_keys)
        self.register_outcomes(["preempt"])
        self._datamodel = {}
        self._onEntry = None
        self._onExit  = None
        self._pause = False
        self._preempt_pause = False
        self._server_name = rospy.get_param('ssm_server_name', '/ssm')
        self._pause_sub = rospy.Subscriber(self._server_name + '/pause',
                                             Bool,
                                             self._request_pause_cb, queue_size=1)
        
        self._preempt_sub = rospy.Subscriber(self._server_name + '/preempt',
                                              Empty,
                                              self._request_preempt_cb, queue_size=1)
        
    def _request_preempt_cb(self, msg):
        """Callback empty message for propagate preempt
        to currently active state.
        """
        self._preempt_pause = True
        
        
    def _request_pause_cb(self, msg):
        """Callback empty message for propagate preempt
        to currently active state.
        """
        
        if(self._is_running or self._pause == True):
                self._pause = msg.data
            
        
    def _update_once(self):
        """Method that updates the state machine once.
        This checks if the current state is ready to transition, if so, it
        requests the outcome of the current state, and then extracts the next state
        label from the current state's transition dictionary, and then transitions
        to the next state.
        """
        outcome = None
        transition_target = None
        last_state_label = self._current_label

        # Make sure the state exists
        if self._current_label not in self._states:
            raise smach.InvalidStateError("State '%s' does not exist. Available states are: %s" %
                    (self._current_label, list(self._states.keys())))

        # Check if a preempt was requested before or while the last state was running
        if self.preempt_requested():
            smach.loginfo("Preempt requested on state machine before executing the next state.")
            # We were preempted
            if self._preempted_state is not None:
                # We were preempted while the last state was running
                if self._preempted_state.preempt_requested():
                    smach.loginfo("Last state '%s' did not service preempt. Preempting next state '%s' before executing..." % (self._preempted_label, self._current_label))
                    # The flag was not reset, so we need to keep preempting 
                    # (this will reset the current preempt)
                    self._preempt_current_state()
                else:
                    # The flag was reset, so the container can reset
                    self._preempt_requested = False
                    self._preempted_state = None
            else:
                # We were preempted after the last state was running
                # So we should preempt this state before we execute it
                self._preempt_current_state()

        # Execute the state

        try:
            self._state_transitioning_lock.release()
            outcome = self._current_state.execute(
                    smach.Remapper(
                        self.userdata,
                        self._current_state.get_registered_input_keys(),
                        self._current_state.get_registered_output_keys(),
                        self._remappings[self._current_label]))
        except smach.InvalidUserCodeError as ex:
            smach.logerr("State '%s' failed to execute." % self._current_label)
            raise ex
        except:
            raise smach.InvalidUserCodeError("Could not execute state '%s' of type '%s': " %
                                             (self._current_label, self._current_state)
                                             + traceback.format_exc())
        finally:
            self._state_transitioning_lock.acquire()
        

        # Check if outcome was a potential outcome for this type of state
        if outcome not in self._current_state.get_registered_outcomes():
            raise smach.InvalidTransitionError(
                    "Attempted to return outcome '%s' preempt_requestedfrom state '%s' of"
                    " type '%s' which only has registered outcomes: %s" %
                    (outcome,
                     self._current_label,
                     self._current_state,
                     self._current_state.get_registered_outcomes()))

        # Check if this outcome is actually mapped to any target
        if outcome not in self._current_transitions:
            raise smach.InvalidTransitionError("Outcome '%s' of state '%s' is not bound to any transition target. Bound transitions include: %s" %
                    (str(outcome), str(self._current_label), str(self._current_transitions)))
        
        # Set the transition target
        transition_target = self._current_transitions[outcome]

        # Check if the transition target is a state in this state machine, or an outcome of this state machine
        if transition_target in self._states:
            # Set the new state 
            self._set_current_state(transition_target)

            # Spew some info
            smach.loginfo("State machine transitioning '%s':'%s'-->'%s'" %
                          (last_state_label, outcome, transition_target))

            # Call transition callbacks
            self.call_transition_cbs()
        else:
            # This is a terminal state
            
            if self._preempt_requested and self._preempted_state is not None:
                if not self._current_state.preempt_requested():
                    self.service_preempt()

            if transition_target not in self.get_registered_outcomes():
                # This is a container outcome that will fall through
                transition_target = outcome

            if transition_target in self.get_registered_outcomes():
                # The transition target is an outcome of the state machine
                self._set_current_state(None)

                # Spew some info
                smach.loginfo("State machine terminating '%s':'%s':'%s'" %
                              (last_state_label, outcome, transition_target))

                # Call termination callbacks
                self.call_termination_cbs([last_state_label],transition_target)

                return transition_target
            else:
                raise smach.InvalidTransitionError("Outcome '%s' of state '%s' with transition target '%s' is neither a registered state nor a registered container outcome." %
                        (outcome, self._current_label, transition_target))
        return None

    def execute(self, parent_ud = smach.UserData()):
        """Run the state machine on entry to this state.
        This will set the "closed" flag and spin up the execute thread. Once
        this flag has been set, it will prevent more states from being added to
        the state machine. 
        """

        # This will prevent preempts from getting propagated to non-existent children

        with self._state_transitioning_lock:
            # Check state consistency
            try:
                self.check_consistency()
            except (smach.InvalidStateError, smach.InvalidTransitionError):
                smach.logerr("Container consistency check failed.")
                return None

            # Set running flag
            self._is_running = True

            # Initialize preempt state
            self._preempted_label = None
            self._preempted_state = None

            # Set initial state 
            self._set_current_state(self._initial_state_label)

            # Copy input keys
            self._copy_input_keys(parent_ud, self.userdata)

            # Spew some info
            smach.loginfo("State machine starting in initial state '%s' with userdata: \n\t%s" %
                    (self._current_label, list(self.userdata.keys())))


            # Call start callbacks
            self.call_start_cbs()

            # Initialize container outcome
            container_outcome = None
            
            ## Copy the datamodel's value into the userData
            for data in self._datamodel:
                if(self._datamodel[data] != ""):
                    self.userdata[data] = self._datamodel[data]
            
            ## Do the <onentry>
            if(self._onEntry is not None):
                try:
                    self._onEntry.execute(self.userdata)
                except Exception as ex:
                    rospy.logerr('%s::onEntry::execute() raised | %s'
                                 %(self.__class__.__name__,str(ex)))
                    return "preempt"
            
            # Step through state machine
            while container_outcome is None and self._is_running and not smach.is_shutdown(): 
                # Update the state machine
                if(self._pause):
                    to_ = rospy.Time.now() - rospy.Duration(5.0)
                    while(self._pause):
                        if(rospy.Time.now() > to_):
                            to_ = rospy.Time.now() + rospy.Duration(5.0)
                            rospy.logwarn("[SSM] : Smart State Machine is paused")
                            rospy.logwarn("[SSM] : Next state executed is : " +self._current_label)
                        rospy.sleep(0.1)
                        if(self._preempt_pause):
                            return "preempt"
  
                container_outcome = self._update_once()
                
                
            ## Do the <onexit>    
            if(self._onExit is not None):
                try:
                    container_outcome = self._onExit.execute(self.userdata,container_outcome)
                except Exception as ex:
                    rospy.logerr('%s::onExit::execute() raised | %s'
                                 %(self.__class__.__name__,str(ex)))
                    return "preempt"

            # Copy output keys
            self._copy_output_keys(self.userdata, parent_ud)    

            # We're no longer running
            self._is_running = False

        return container_outcome

    
class ssmMainStateMachine(ssmStateMachine):
    """StateMachine
    
    This is a finite state machine smach container. Note that though this is
    a state machine, it also implements the L{smach.State}
    interface, so these can be composed hierarchically, if such a pattern is
    desired.

    States are added to the state machine as 3-tuple specifications:
     - label
     - state instance
     - transitions

    The label is a string, the state instance is any class that implements the
    L{smach.State} interface, and transitions is a dictionary mapping strings onto
    strings which represent the transitions out of this new state. Transitions
    can take one of three forms:
     - OUTCOME -> STATE_LABEL
     - OUTCOME -> None (or unspecified)
     - OUTCOME -> SM_OUTCOME
    """
    
    def __init__(self, outcomes):
        ssmStateMachine.__init__(self,outcomes)
        self._input_keys=[]
        self._output_keys=[]
        
    def _request_preempt_cb(self, msg):
        """Callback empty message for propagate preempt
        to currently active state.
        """
        if(self._pause):
            self._preempt_pause = True
        else:
            self.request_preempt()
    
    def execute(self, parent_ud = smach.UserData()):
        """Run the state machine on entry to this state.
        This will set the "closed" flag and spin up the execute thread. Once
        this flag has been set, it will prevent more states from being added to
        the state machine. 
        """
        if(self.preempt_requested()):
            return 'preempt'
        ##Create a the log file if ssm_enable_log is True
        if(rospy.get_param("ssm_enable_log", False) == True):
            date_str = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_")
            self.userdata.logfile = rospy.get_param("ssm_log_path", default="/tmp/") + date_str +"log.txt"
            try:
                file = open(self.userdata.logfile, "a")
                file.write(StrTimeStamped() +"SMART STATE MACHINE Started \n")
                file.close()
            except:
                rospy.logerr("Log Fail !")
                return 'preempt'
    
        container_outcome = super(ssmMainStateMachine,self).execute(parent_ud)
        
        if(rospy.get_param("ssm_enable_log", False) == True):
            file = open(self.userdata.logfile, "a")
            file.write(StrTimeStamped() +"SMART STATE MACHINE Finished \n")
            file.close()
        
        return container_outcome        
