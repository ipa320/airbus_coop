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

from std_msgs.msg import String, Empty

import threading
import smach
import rospy
import os
import airbus_ssm_core.msg
import pydot
from ast import literal_eval
from pydot import Subgraph


class ssmGraph(object):
    def __init__(self, state_machine, action_server = None):
        """Traverse the smach tree starting at root, and construct introspection
        proxies for getting and setting debug state."""
        self._state_machine = state_machine
        self._graph_dot = pydot.Dot(graph_type = 'digraph', compound='true', newrank='true', minlen='0', size='"1,2"')
        self._level = 0
        
        self._rank_nodes = {}
        self._parent_nodes = []
        self._parallel_nodes = []
        self._parallel_starting = []
        self._parallel_ending = []
        self._server_name = rospy.get_param('ssm_server_name', '/ssm')
        self._update_pub = rospy.Publisher(self._server_name + "/ssm_dotcode", String, queue_size = 1)
        self._sub_update = None
        
        
    def _update_cb(self, msg):
        statut_dict = literal_eval(msg.data)
        self.update_color('ROOT', statut_dict,self._graph_dot)
        self.send_graph()
    
    def update_color(self, name, statut_dict, graph): 
        for state_ in statut_dict[name]:
            if(state_ in self._parent_nodes):
                if(statut_dict[name][state_] == 1):
                    glevel = 1
                    for level in self._rank_nodes:
                        if(state_ in self._rank_nodes[level]):
                            glevel = int(level)
                            if(glevel < 1):
                                glevel = 1
                            if(glevel > 4):
                                glevel = 4
                    color = 'lightblue'+str(glevel)
                elif(statut_dict[name][state_] == 0):
                    color = 'white'
                else:
                    color = 'yellow2'
                    
                if(len(graph.get_subgraph('cluster' + str(state_)))>0):
                    graph.get_subgraph('cluster' + str(state_))[0].set('style','filled')
                    graph.get_subgraph('cluster' + str(state_))[0].set('fillcolor',color)
                    self.update_color(state_, statut_dict, graph.get_subgraph('cluster' + state_)[0])
                elif(len(graph.get_subgraph('"cluster' + state_+'"'))>0):
                    graph.get_subgraph('"cluster' + state_+'"')[0].set('style','filled')
                    graph.get_subgraph('"cluster' + state_+'"')[0].set('fillcolor',color)
                    self.update_color(state_, statut_dict, graph.get_subgraph('"cluster' + state_+'"')[0])
                else:
                    pass
                
            else:
                if(statut_dict[name][state_] == 1):
                    color = 'greenyellow'
                elif(statut_dict[name][state_] == 0):
                    color = 'white'
                else:
                    color = 'orange'
                if(len(graph.get_node(state_))>0):
                    graph.get_node(state_)[0].set('style','filled')
                    graph.get_node(state_)[0].set('fillcolor',color)
                elif(len(graph.get_node('"'+state_+'"'))>0):
                    graph.get_node('"'+state_+'"')[0].set('style','filled')
                    graph.get_node('"'+state_+'"')[0].set('fillcolor',color)
                else:
                    pass
            
        
    def start(self):
        # Construct proxies
        self.construct_subgraph(self._state_machine, 'ROOT', self._graph_dot)
        self.layout_nodes(self._graph_dot)
        self.send_graph()
        self._sub_update = rospy.Subscriber(self._server_name + "/ssm_status", String, self._update_cb, queue_size = 1)
         
    def send_graph(self):
        Msg = String()
        Msg.data = self._graph_dot.to_string()
        self._update_pub.publish(Msg)
    
    def stop(self):
        if(self._sub_update is not None):
            self._sub_update.unregister()
        
    def construct_subgraph(self, state, name, graph):
        """Recursively construct graph."""
        if isinstance(state, smach.concurrence.Concurrence):
            self.add_final(state, name, graph)
        
        for (label, child) in state.get_children().items():
            # If this is also a container, recurse into it
            if(str(self._level) not in self._rank_nodes):
                self._rank_nodes[str(self._level)] = []
            self._rank_nodes[str(self._level)].append(label)
            if isinstance(child, smach.container.Container):
                self._parent_nodes.append(label)
                new_graph = pydot.Subgraph('cluster'+label, label=label)
                graph.add_subgraph(new_graph)       
                if isinstance(child, smach.concurrence.Concurrence):
                    self._parallel_nodes.append(label)
                self._level = self._level +1
                self.construct_subgraph(child, label, new_graph)
                self._level = self._level -1
            else:
                graph.add_node(pydot.Node(label, label=label))
                
        self.add_initial(state, name, graph)
        if not(isinstance(state, smach.concurrence.Concurrence)):
            self.add_final(state, name, graph)
        self.create_edges(state, name, graph)

        
    def create_edges(self, state, name, graph):
        from_label = ''
        to_label = ''
        args_t = ''
        #init
        node_name = 'init_'+ name
        if isinstance(state, smach.state_machine.StateMachine):  
            if(state._initial_state_label in self._parent_nodes): #transition from parent node
                _edge = pydot.Edge(node_name,'init_' + state._initial_state_label)
                _edge.set('lhead', 'cluster'+state._initial_state_label)
                graph.add_edge(_edge)
            else:
                graph.add_edge(pydot.Edge(node_name,state._initial_state_label))
        else:
            for (label, child) in state.get_children().items():
            # If this is also a container, recurse into it
                if isinstance(child, smach.container.Container):
                    graph.add_edge(pydot.Edge(node_name,'init_' + label, style='invis'))
                else:
                    graph.add_edge(pydot.Edge(node_name,label, style='invis'))
            
        for transition in state.get_internal_edges():
            
            if(transition[0] != 'preempt'):
                _edge = pydot.Edge('""','""',label=transition[0])
                if(transition[0]==transition[2]): #final
                    if(('"parrallel_'+name+'_'+transition[2]+'"') in self._parallel_ending):
                        to_label = '"parrallel_'+name+'_'+transition[2]+'"'
                    else:
                        to_label = 'final_' + name+'_'+transition[0]
                else:
                    if (transition[2] in self._parent_nodes): #transition to parent node
                        to_label = 'init_' + transition[2]
                        _edge.set('lhead', 'cluster'+transition[2])
                    else:
                        if(('"parrallel_'+name+'_'+transition[2]+'"') in self._parallel_ending):
                            to_label = '"parrallel_'+name+'_'+transition[2]+'"'
                        else:
                            to_label = transition[2]

                if(transition[1] in self._parent_nodes): #transition from parent node
                    from_label = 'final_'+ transition[1] + '_'+transition[0]
                    if(transition[1] in self._parallel_nodes):
                        from_label = 'parrallel_'+transition[1]+'_'+transition[0]
                    _edge.set('ltail', 'cluster'+transition[1])
                else:
                    from_label = transition[1]
                new_edge = pydot.Edge(from_label,to_label)#,_edge.get_attributes())
                for attr_ in _edge.get_attributes():
                    new_edge.set(attr_, _edge.get_attributes()[attr_]) 
                graph.add_edge(new_edge)
        
    def add_final(self, state, state_label, graph):
        if(isinstance(state, smach.concurrence.Concurrence)):
            for transition in state.get_internal_edges():
                if(transition[0] != 'preempt'):
                    node_name = 'parrallel_'+state_label+'_'+transition[2]
                    self._parallel_ending.append('"'+node_name+'"')
                    graph.add_node(pydot.Node(node_name,shape='underline',margin='0',height='0.2',label=transition[2]))
        else:
            for transition in state.get_internal_edges():
                if(transition[0] != 'preempt'):
                    if(transition[0]==transition[2]): #final
                        node_name = 'final_'+ state_label + '_'+transition[0]
                        graph.add_node(pydot.Node(node_name,shape='doublecircle',style='filled',fillcolor='black',width='0.25',label='""'))
        
    def add_initial(self, state, state_label, graph): 
        node_name = 'init_'+ state_label
        if isinstance(state, smach.state_machine.StateMachine):
            graph.add_node(pydot.Node(node_name,shape='circle', style='filled',fillcolor='black',width='0.25',label='""'))
        else:
            graph.add_node(pydot.Node(node_name,shape='box',margin='0',width='0', height='0', fixedsize='true',style='invis',label='""'))
            self._parallel_starting = [node_name]
            
    def layout_nodes(self, graph):
        subgraph_init = pydot.Subgraph(rank='same')
        for node in graph.get_node_list():
            if("init_" in node.get_name()):
                if(node.get_name() in self._parallel_starting):
                    for edge_ in graph.get_edge_list():
                        if(edge_.get_source()==node.get_name()):
                            subgraph_init.add_node(pydot.Node(edge_.get_destination()))
                else:
                    subgraph_init.add_node(node)
     
            
        subgraph_final = pydot.Subgraph(rank='same')
        for node in graph.get_node_list(): 
            if("final_" in node.get_name()):
                subgraph_final.add_node(node)
            if(node.get_name() in self._parallel_ending):
                subgraph_final.add_node(node)
        graph_list = graph.get_subgraph_list()
        graph.add_subgraph(subgraph_init)
        graph.add_subgraph(subgraph_final)
        for graph_ in graph_list:
            self.layout_nodes(graph_)
