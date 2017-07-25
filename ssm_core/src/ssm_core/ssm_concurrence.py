#!/usr/bin/env python

import threading
import traceback
import copy
from contextlib import contextmanager

import smach
import rospy

__all__ = ['Concurrence']

class ssmConcurrence(smach.Concurrence):
    """Concurrence Container

    This state allows for simple split-join concurrency. The user adds a set of
    states which are all executed simultaneously. The concurrent split state
    can only transition once all conatained states are ready to transition.
    
    This container can be configured to return a given outcome as a function of
    the outcomes of the contained states. This is specified in the constructor
    of the class, or after construction with L{Concurrence.add_outcome_map}.

    While a concurrence will not terminate until all if its children terminate,
    it is possible for it to preempt a subset of states 
     - All child states terminate
     - At least one child state terminates
     - A user-defined callback signals termination

    Given these causes of termination, the outcome can be determined in four ways:
     - A user-defined callback returns an outcome
     - A child-outcome map which requires ALL states to terminate is satisfied
     - A child-outcome map which requires ONE state to terminate is satisfied
     - No  maps are satisfied, so the default outcome is returned

    The specification of the outcome maps and the outcome callback are
    described in the constructor documentation below. More than one policy can
    be supplied, and each policy has the potential to not be satisfied. In the
    situation in which multiple policies are provided, and a given policy is
    not satisfied, the outcome choice precedence is as follows:
     - Outcome callback
     - First-triggered outcome map
     - last-triggered outcome map
     - Default outcome

    In practive it is best to try to accomplish your task with just ONE outcome
    policy.

    """
    def __init__(self,
            outcomes,
            input_keys = [],
            output_keys = [],
            outcome_map = {},
            outcome_cb = None,
            child_termination_cb = None
            ):

        input_keys.append("logfile")
        output_keys.append("logfile")
        outcomes.append('preempt')
        default_outcome = 'preempt'
        smach.Concurrence.__init__(self, outcomes, default_outcome, input_keys, output_keys, outcome_map, outcome_cb, child_termination_cb)
        self._datamodel = {}
        self._onEntry = None
        self._onExit  = None
        self._tree_view = None
        self._tree_view_Lock = threading.Lock()
        self._tree_view_cb = None
        
    ### State interface
    def execute(self, parent_ud = smach.UserData()):
        """Overridden execute method.
        This starts all the threads.
        """
        # Clear the ready event
        self._ready_event.clear()
        # Reset child outcomes
        self._child_outcomes = {}
        
        # Copy input keys
        self._copy_input_keys(parent_ud, self.userdata)
        
        ## Copy the datamodel's value into the userData
        for data in self._datamodel:
            self.userdata[data] = self._datamodel[data]
        
        ## Do the <onentry>
        if(self._onEntry is not None):
            try:
                self._onEntry.execute(self.userdata)
            except Exception as ex:
                rospy.logerr('%s::onEntry::execute() raised | %s'
                             %(self.__class__.__name__,str(ex)))
                return "preempt"

        # Spew some info
        smach.loginfo("Concurrence starting with userdata: \n\t%s" %
                (str(list(self.userdata.keys()))))

        # Call start callbacks
        self.call_start_cbs()

        # Create all the threads
        for (label, state) in ((k,self._states[k]) for k in self._states):
            # Initialize child outcomes
            
            self._child_outcomes[label] = None
            self._threads[label] = threading.Thread(
                    name='concurrent_split:'+label,
                    target=self._state_runner,
                    args=(label,))

        # Launch threads
        for thread in self._threads.values():
            thread.start()
        
        # Wait for done notification
        self._done_cond.acquire()
        
        # Notify all threads ready to go
        self._ready_event.set()
        
        # Wait for a done notification from a thread
        self._done_cond.wait()
        self._done_cond.release()

        # Preempt any running states
        smach.logdebug("SMACH Concurrence preempting running states.")
        for label in self._states:
            if self._child_outcomes[label] == None:
                self._states[label].request_preempt()

        # Wait for all states to terminate
        while not smach.is_shutdown():
            if all([not t.isAlive() for t in self._threads.values()]):
                break
            self._done_cond.acquire()
            self._done_cond.wait(0.1)
            self._done_cond.release()

        # Check for user code exception
        if self._user_code_exception:
            self._user_code_exception = False
            raise smach.InvalidStateError("A concurrent state raised an exception during execution.")

        # Check for preempt
        if self.preempt_requested():
            # initialized serviced flag
            children_preempts_serviced = True

            # Service this preempt if 
            for (label,state) in ((k,self._states[k]) for k in self._states):
                if state.preempt_requested():
                    # Reset the flag
                    children_preempts_serviced = False
                    # Complain
                    smach.logwarn("State '%s' in concurrence did not service preempt." % label) 
                    # Recall the preempt if it hasn't been serviced
                    state.recall_preempt()
            if children_preempts_serviced:
                smach.loginfo("Concurrence serviced preempt.")
                self.service_preempt()

        # Spew some debyg info
        smach.loginfo("Concurrent Outcomes: "+str(self._child_outcomes))

        # Initialize the outcome
        outcome = self._default_outcome

        # Determine the outcome from the outcome map
        smach.logdebug("SMACH Concurrence determining contained state outcomes.")
        for (container_outcome, outcomes) in ((k,self._outcome_map[k]) for k in self._outcome_map):
            if all([self._child_outcomes[label] == outcomes[label] for label in outcomes]):
                smach.logdebug("Terminating concurrent split with mapped outcome.")
                outcome = container_outcome

        # Check outcome callback
        if self._outcome_cb:
            try:
                cb_outcome = self._outcome_cb(copy.copy(self._child_outcomes))
                if cb_outcome:
                    if cb_outcome == str(cb_outcome):
                        outcome = cb_outcome
                    else:
                        smach.logerr("Outcome callback returned a non-string '%s', using default outcome '%s'" % (str(cb_outcome), self._default_outcome))
                else:
                    smach.logwarn("Outcome callback returned None, using outcome '%s'" % outcome)
            except:
                raise smach.InvalidUserCodeError(("Could not execute outcome callback '%s': " % self._outcome_cb)+traceback.format_exc())

        # Cleanup
        self._threads = {}
        self._child_outcomes = {}

        # Call termination callbacks
        self.call_termination_cbs(list(self._states.keys()), outcome)
        
        ## Do the <onexit>    
        if(self._onExit is not None):
            try:
                outcome = self._onExit.execute(self.userdata,outcome)
            except Exception as ex:
                rospy.logerr('%s::onExit::execute() raised | %s'
                             %(self.__class__.__name__,str(ex)))
                return "preempt"

        # Copy output keys
        self._copy_output_keys(self.userdata, parent_ud)

        return outcome


    def _state_runner(self,label):
        """Runs the states in parallel threads."""

        # Wait until all threads are ready to start before beginnging
        self._ready_event.wait()
        
        self.call_transition_cbs()

        # Execute child state
        
        self._tree_view_enable_state(label)
        
        try:
            self._child_outcomes[label] = self._states[label].execute(smach.Remapper(
                self.userdata,
                self._states[label].get_registered_input_keys(),
                self._states[label].get_registered_output_keys(),
                self._remappings[label]))
        except:
            self._user_code_exception = True
            with self._done_cond:
                self._done_cond.notify_all()
            raise smach.InvalidStateError(("Could not execute child state '%s': " % label)+traceback.format_exc())
        
        self._tree_view_disable_state(label)
        # Make sure the child returned an outcome
        if self._child_outcomes[label] is None:
            raise smach.InvalidStateError("Concurrent state '%s' returned no outcome on termination." % label)
        else:
            smach.loginfo("Concurrent state '%s' returned outcome '%s' on termination." % (label, self._child_outcomes[label]))

        # Check if all of the states have completed
        with self._done_cond:
            # initialize preemption flag
            preempt_others = False
            # Call transition cb's
            self.call_transition_cbs()
            # Call child termination cb if it's defined
            if self._child_termination_cb:
                try:
                    preempt_others = self._child_termination_cb(self._child_outcomes)
                except:
                    raise smach.InvalidUserCodeError("Could not execute child termination callback: "+traceback.format_exc())
            ## Check if we have finished one outcome
            for (container_outcome, outcomes) in ((k,self._outcome_map[k]) for k in self._outcome_map):
                if all([self._child_outcomes[label] == outcomes[label] for label in outcomes]):
                    preempt_others = True

            # Notify the container to terminate (and preempt other states if neceesary)
            if preempt_others or all([o is not None for o in self._child_outcomes.values()]):
                self._done_cond.notify_all()
        
        
    def _create_tree_view(self):
        self._tree_view = {}
        for child in self.get_children():
            self._tree_view[child]  = 0
            
    def _tree_view_enable_state(self, label):
        if(self._tree_view is not None):
            self._tree_view_Lock.acquire()
            self._tree_view[label] = 1
            self.call_update_tree_view_cb()
            self._tree_view_Lock.release()
            
    
    def _tree_view_disable_state(self, label):
        if(self._tree_view is not None):
            self._tree_view_Lock.acquire()
            self._tree_view[label] = 0
            self.call_update_tree_view_cb()
            self._tree_view_Lock.release()
            
    
    def get_tree_view(self):
        return self._tree_view

    def register_tree_view_cb(self, callback):
        self._tree_view_cb = callback
    
    def call_update_tree_view_cb(self):
        if(self._tree_view_cb is not None):
            try:
                self._tree_view_cb()
            except:
                smach.logerr("Could not execute treeview callback: "+traceback.format_exc())
