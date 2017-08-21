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



import ssm_state
import ssm_state_machine
import ssm_concurrence
import ssm_on
import ssm_skill_provider

import rospy
import smach_ros
import re


from roslib.packages import get_pkg_dir
import xml.etree.ElementTree as etree

# RE
RE_PREFIXED_TAG = "$"
RE_PREFIXED_BEGIN_TAG = "${"
RE_PREFIXED_END_TAG = "}"

def get_pkg_dir_from_prefix(path, ros_package_path=None):
    
    if RE_PREFIXED_TAG not in path:
        return path
    
    splitpath = path.split(RE_PREFIXED_END_TAG)
    after_prefix_path = splitpath[-1]
    prefix = splitpath[0].split(RE_PREFIXED_BEGIN_TAG)[-1]
    rpath = get_pkg_dir(prefix)+after_prefix_path
    
    return rpath

class EmptyState(ssm_state.ssmState):
    def __init__(self):
        ssm_state.ssmState.__init__(self,outcomes=["next"])
        
    def execution(self, ud):
        ud.skill = "Empty"
        return "next"
        

class ssmInterpreter:
    
    def __init__(self, file):
        self.StateDict =  None
        file_ = open(file).read()
        file_ = re.sub(' xmlns="[^"]+"', '', file_, count=1)
        self.root = etree.fromstring(file_)
        self.skillProvider = None
        self.CheckBool = True
        if(self.root is None):
            rospy.logerr("[SCXML Interpreter] : Error in root !!!!")
            self.CheckBool = False
        else:
            pass

    def readFile(self):
        '''
            First we import the skills
            Then we construct the state machine
            We return the constructed state machine
        '''
        
        skill = self.root.find("datamodel/data[@id='skill_file']")
        if(skill is None):
            rospy.logerr("[SCXML Interpreter] : No Skill Register found !!")
            self.CheckBool = False
            return
        else:
            self.skillProvider = ssm_skill_provider.SkillProvider(get_pkg_dir_from_prefix(str(skill.attrib['expr'])))
            
        return self.translateSCXMLtoSSM()

    def translateSCXMLtoSSM(self):       
        '''
            Read the datamodel. This is the list of all datas used in the state machine.
        '''
        State = EmptyState()
        #Find the outcomes of the top state machine
        outcomes_=[]
        final = self.root.find('./final') 
        if(final == None):
            rospy.logerr("[SCXML Interpreter] : No final State found in the SCXML ! You must have one !")
            self.CheckBool = False
            return
        current_final = final.attrib.get('id')
        for transition in self.root.findall("./*/transition[@target='"+str(current_final)+"']"):
            event = transition.attrib.get('event')
            if(event is None):
                rospy.logerr("[SCXML Interpreter] : No Transition Event for %s !!!!" % ID)
                self.CheckBool = False
                return
            target = transition.attrib.get('target')
            if(target is None):
                rospy.logerr("[SCXML Interpreter] : No Transition Target for %s !!!!" % ID)
                self.CheckBool = False
                return
            if(target == current_final):
                target = event
            outcomes_.append(event)
        ##Init the new smach state machine
        SSM = ssm_state_machine.ssmMainStateMachine(outcomes = outcomes_)
        SSM.userdata.logfile = ""
        '''
            Setup the data from the scxml
        '''
        datamodel = self.root.find('datamodel')
        if(datamodel is None):
            rospy.logwarn("[SCXML Interpreter] : No Datamodel detected !!!!")
        else:
            for data in datamodel:
                data_ID = data.attrib['id']
                data_expr = data.attrib['expr']
                SSM.userdata[data_ID] = data_expr
                
        '''
            Read the states in the scxml
            If there is a problem in a state the interpreter will return and failed
        '''
        
        ##Setupt the initial state for the intial state machine
        if(self.root.attrib.get('initial') == None):
            rospy.logerr("[SCXML Interpreter] : No Initial State in the SCXML !!!!" )
            self.CheckBool = False
            return
        SSM._initial_state_label = self.root.attrib.get('initial')
        ##Init variables for interations
        finish_ = False
        list_parent = ['.']
        list_SM = [SSM]
        while(finish_ == False):
            ##Construct the actual level
            _list_parent=[]
            _list_SM = [] 
            for i_parent in range(len(list_parent)):

                parent = list_parent[i_parent]
                current_SM = list_SM[i_parent]
                current_SM.open()
                ##Check the final state
                if(type(current_SM) is not ssm_concurrence.ssmConcurrence):
                    final = self.root.find(parent+'/final') 
                    if(final == None):
                        rospy.logerr("[SCXML Interpreter] : No final State found in "+parent+" !!!!")
                        self.CheckBool = False
                        return
                    final_id = final.attrib.get('id')
                    
                list_state = self.root.findall(parent+'/state')
                list_parallel = self.root.findall(parent+'/parallel')
                for parrallel in list_parallel:
                    
                    ID = parrallel.attrib.get('id')
                    if(ID is None):
                        rospy.logerr("[SCXML Interpreter] : No ID for a State !!!!" )
                        self.CheckBool = False
                        return
                    ##get onEntry and onExit
                    onEntry = self.get_on_entry(parrallel)
                    onExit = self.get_on_exit(parrallel)
                    ##add to the parent list
                    _list_parent.append(parent+"/parallel[@id='"+str(ID)+"']")
                     ##Find the transitions
                    transitions_ = {}
                    outcomes_ = []
                    outcomes_map = {}

                    for transition in parrallel.findall('transition'):                       
                        event = transition.attrib.get('event')
                        if(event is None):
                            rospy.logerr("[SCXML Interpreter] : No Transition Event for %s !!!!" % ID)
                            self.CheckBool = False
                            return
                        target = transition.attrib.get('target')
                        if(target is None):
                            rospy.logerr("[SCXML Interpreter] : No Transition Target for %s !!!!" % ID)
                            self.CheckBool = False
                            return
                        cond = transition.attrib.get('cond')
                        if(target is None):
                            rospy.logerr("[SCXML Interpreter] : No Transition Condition  for %s !!!!" % ID)
                            self.CheckBool = False
                            return
                        if(target == final_id):
                            target = event
  
                        map_list = self.convertToConcurenceMap(cond)
                        if(len(map_list) == 0):
                            rospy.logerr("[SCXML Interpreter] : Error during the convertion event to map for %s !!!!" % ID)
                            self.CheckBool = False
                            return 
                        for map_ in map_list:
                            outcome_ = event
                            outcomes_.append(outcome_)
                            outcomes_map[outcome_] = map_
                            transitions_[outcome_] = target                                      
 
                    datamodel_ = {}
                    ##Add the datamodel
                    for data in parrallel.findall('./datamodel/data'):  
                        data_ID = data.attrib['id']
                        data_expr = data.attrib['expr']
                        datamodel_[data_ID] = data_expr
                        if(not(SSM.userdata.__contains__(data_ID))):
                            SSM.userdata.__setattr__(data_ID, data_expr)

                    ##Find the io_keys
                    keys_ = self.findIOkeys(parrallel)
  
                    newSM = ssm_concurrence.ssmConcurrence(outcomes_,outcome_map=outcomes_map,input_keys = keys_, output_keys = keys_) ##Define the empty concurence
                    newSM._datamodel = datamodel_   
                    newSM._onEntry = onEntry
                    newSM._onExit = onExit
 
                    self.addToSM(current_SM,ID,newSM,transitions_,keys_)

                    _list_SM.append(newSM)

                for state in list_state:

                    ID = state.attrib.get('id')
                    if(ID is None):
                        rospy.logerr("[SCXML Interpreter] : No ID for a State !!!!" )
                        self.CheckBool = False
                        return         
                    ##get onEntry and onExit
                    onEntry = self.get_on_entry(state)
                    onExit = self.get_on_exit(state)
                                
                    if(state.find('state') is not None):
                        ##State is parent ==> it's a state machine
                        
                        ##add to the parent list
                        _list_parent.append(parent+"/state[@id='"+str(ID)+"']")
                        ##Find the transitions
                        transitions_ = {}
                        outcomes_ = []
                        if(type(current_SM) is not ssm_concurrence.ssmConcurrence):
                            for transition in state.findall('transition'):
                                event = transition.attrib.get('event')
                                if(event is None):
                                    rospy.logerr("[SCXML Interpreter] : No Transition Event for %s !!!!" % ID)
                                    self.CheckBool = False
                                    return
                                target = transition.attrib.get('target')
                                if(target is None):
                                    rospy.logerr("[SCXML Interpreter] : No Transition Target for %s !!!!" % ID)
                                    self.CheckBool = False
                                    return
                                if(target == final_id):
                                    target = event
                                    
                                outcomes_.append(event)
                                transitions_[event] = target
                        else:
                            final = state.find('final') 
                            if(final == None):
                                rospy.logerr("[SCXML Interpreter] : No final State found in "+state+" !!!!")
                                self.CheckBool = FalseINFO
                                return
        
                            current_final = final.attrib.get('id')
                            for transition in state.findall("./state/transition[@target='"+str(current_final)+"']"):
                                event = transition.attrib.get('event')
                                if(event is None):
                                    rospy.logerr("[SCXML Interpreter] : No Transition Event for %s !!!!" % ID)
                                    self.CheckBool = False
                                    return
                                target = transition.attrib.get('target')
                                if(target is None):
                                    rospy.logerr("[SCXML Interpreter] : No Transition Target for %s !!!!" % ID)
                                    self.CheckBool = False
                                    return
                                if(target == current_final):
                                    target = event
                                outcomes_.append(event)
                                transitions_[event] = target
                            
                        
                        
                        ##Add the datamodel
                        datamodel_ = {}
                        for data in state.findall('./datamodel/data'):  
                            data_ID = data.attrib['id']
                            data_expr = data.attrib['expr']
                            datamodel_[data_ID] = data_expr
                            if(not(SSM.userdata.__contains__(data_ID))):
                                SSM.userdata.__setattr__(data_ID, data_expr)
                            
                        ##Find the io_keys
                        keys_ = self.findIOkeys(state)
                        
                        ##Add the intial state
                        initial = state.find("initial")
                        if(initial is None):
                            rospy.logerr("[SCXML Interpreter] : No Initial state for a Parent State !!!!" )
                            self.CheckBool = False
                            return
                        
                        newSM = ssm_state_machine.ssmStateMachine(outcomes_, input_keys = keys_, output_keys = keys_) ##Define the empty state machine
                        newSM._datamodel = datamodel_
                        newSM._onEntry = onEntry
                        newSM._onExit = onExit
                        
                        newSM._initial_state_label = str(initial.find('transition').attrib.get('target'))
                        self.addToSM(current_SM,ID,newSM,transitions_, keys_)
                        _list_SM.append(newSM)
                        
                    else:
                        ##State is not a compound state so it's a state

                        ##Find the transitions
                        transitions_ = {}
                        for transition in state.findall('transition'):
                            event = transition.attrib.get('event')
                            if(event is None):
                                rospy.logerr("[SCXML Interpreter] : No Transition Event for %s !!!!" % ID)
                                self.CheckBool = False
                                return
                            target = transition.attrib.get('target')
                            if(target is None):
                                rospy.logerr("[SCXML Interpreter] : No Transition Target for %s !!!!" % ID)
                                self.CheckBool = False
                                return
                            if(target == final_id):
                                target = event
                            transitions_[event] = target
                        
                        
                            
                        datamodel_ = {}
                        ##Add the datamodel
                        for data in state.findall('./datamodel/data'):
                            data_ID = data.attrib['id']
                            data_expr = data.attrib['expr']
                            datamodel_[data_ID] = data_expr
                            if(not(SSM.userdata.__contains__(data_ID))):
                                SSM.userdata.__setattr__(data_ID, data_expr)
                        
                        ##Find the skill
                        skill_name = state.find("./datamodel/data[@id='skill']")
                        if(skill_name is not None):
                            try:
                                State = self.skillProvider.load(skill_name.attrib.get('expr'))()
                            except Exception as ex:
                                rospy.logerr('Import fail from Skill "%s" !'%name)
                                rospy.logerr(ex)
                                return None
                        else:
                            State = EmptyState()
                        keys_ = self.findIOkeys(state)   
                        State.register_io_keys(keys_) 
                        State._datamodel = datamodel_   
                        State._onEntry = onEntry
                        State._onExit = onExit
                        self.addToSM(current_SM,ID,State,transitions_,keys_)
                        for keys in State.get_registered_input_keys():
                            if(not(SSM.userdata.__contains__(keys))):
                                SSM.userdata.__setattr__(keys, None)
            ##Current level is finished
            if(len(_list_parent)==0):##There is no lower level
                finish_ = True
            else:
                list_parent = _list_parent ##copy the new list of parent
                list_SM = _list_SM ##copy the list of State Machine
            
        return SSM
    
    def addToSM(self,current_SM,added_ID, added_State, transitions_,keys_):
        '''
        Add a state, a state machine or a concurence to the current state machine / concurence
        '''
        remapping_ = {'logfile':'logfile'}
        for keys in keys_:  
            remapping_[keys] = keys
        if(type(current_SM) is ssm_state_machine.ssmStateMachine or type(current_SM) is ssm_state_machine.ssmMainStateMachine):
            current_SM.add(added_ID,added_State,transitions_,remapping_)
        elif(type(current_SM) is ssm_concurrence.ssmConcurrence):
            current_SM.add(added_ID,added_State,remapping_)
    
                
    def convertToConcurenceMap(self, event):
        '''
        Split the event in the transition into a map for the concurence purpose
        For example :
        A/Succeeded.B/Failed ==> [{'A':'Succeeded', 'B':'Failed'}]
        A/Succeeded + B/Failed ==> [{'A':'Succeeded'}, {'B':'Failed'}]
        A/Failed ==> [{'A':'Failed'}]
        '''
        map_list = []
        if(event.find(' AND ') == -1): ##There is no "and" condition
            if(event.find(' OR ') == -1): ## There is no "or" condition
                list_ = event.split('.')
                map_list.append({list_[0]:list_[1]})
            else:
                list_or = event.split(' OR ')
                for event_or in list_or:
                    list_ = event_or.split('.')
                    map_list.append({list_[0]:list_[1]})
        else:
            list_and = event.split(' AND ')
            map_ = {}
            for event_and in list_and:
                list_and = event_and.split('.')
                map_[list_and[0]] = list_and[1]
            map_list.append(map_)
        
        return map_list
        
    def get_on_entry(self, state):
        
        if(state.find('onentry') == None):
            return None
        else:
            logs = {}
            script = None
            if(state.find('./onentry/script') is not None):
                script = state.find('./onentry/script').text
            
            for log in state.findall('./onentry/log'):
                logs[log.attrib['label']] = log.attrib['expr']
            
            return ssm_on.onEntry(logs, script)
            
        
    def get_on_exit(self, state):
        
        if(state.find('onexit') == None):
            return None
        else:
            logs = {}
            script = None
            if(state.find('./onexit/script') is not None):
                script = state.find('./onexit/script').text
            
            for log in state.findall('./onexit/log'):
                logs[log.attrib['label']] = log.attrib['expr']
            
            return ssm_on.onExit(logs, script)
        
    def findIOkeys(self, state):
        io_keys = set()
        for data in state.findall('.//datamodel/data'):
            ##Grab all data ID
            io_keys.add(data.attrib['id'])
            ##Grad all skills
            if(data.attrib['id'] == 'skill'):
            ##Find the skill
                if(data.attrib['expr'] is not None):
                    try:
                        State = self.skillProvider.load(data.attrib['expr'])()
                    except Exception as ex:
                        rospy.logerr('Import fail from Skill "%s" !'%data.attrib['expr'])
                        rospy.logerr(ex)
                        return None
                else:
                    State = EmptyState()
                for key in State.get_registered_input_keys():
                    io_keys.add(str(key))
                
        for log in state.findall('.//onentry/log'):
            ##Grab all data ID
            if(not(log.attrib['label'] == "") and not(log.attrib['label'] == 'outcome')):
                io_keys.add(log.attrib['label'])
        for log in state.findall('.//onexit/log'):
            ##Grab all data ID
            if(not(log.attrib['label'] == "") and not(log.attrib['label'] == 'outcome')):
                io_keys.add(log.attrib['label']) 
                
        return list(io_keys)
    
    
        
    

        
