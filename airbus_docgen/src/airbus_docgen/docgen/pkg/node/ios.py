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

from airbus_docgen import env
from airbus_docgen.common import html
from airbus_docgen.common.html import HtmlElement

from airbus_docgen.digraph.digraph import *
from airbus_docgen.digraph.model.actionlib import ActionServerModel, ActionClientModel
from airbus_docgen.digraph.model.topic import SubscribersModel, PublishersModel

class NodeInputOutput(HtmlElement):
    
    def __init__(self):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"io-node"})
        
    def _create_digraph(self, name):
        
        digraph = Digraph("IoGraph")
        digraph.setAttrib(Digraph.NODESEP, 0.8)
        digraph.setAttrib(Digraph.RANKDIR, 'LR')
         
        nconf = NODE("node")
        nconf.setAttrib(NODE.SHAPE, SHAPE.Plaintext)
        digraph.addNode(nconf)
        
        pkg = NODE(name)
        pkg.setAttrib(NODE.SHAPE, SHAPE.Ellipse)
        pkg.setAttrib(NODE.STYLE, STYLE.FILLED)
        pkg.setAttrib(NODE.COLOR, RgbColor.CornflowerBlue)
        pkg.setAttrib(NODE.FONTSIZE, 22)
        digraph.addRootNode(pkg)
        
        return digraph
        
    def read(self, node_name, node_xml):
        
        digraph = self._create_digraph(node_name)
        
        for child in node_xml.find('io'):
            if child.tag == "topics":
                try:
                    self._read_topics(child, digraph)
                except:
                    continue
            elif child.tag == "actionlib":
                try:
                    self._read_actionlibs(child, digraph)
                except:
                    continue
            elif child.tag == "services":
                try:
                    self._read_services(child, digraph)
                except:
                    continue
            else:
                html.HTMLException("Unknown io tag '%s' from node %s !"%(child.tag, node_name), self)
        
        digraph.saveDot(env.ROSDOC_DOT+"/io/%s.dot"%node_name)
        digraph.dotToPng(env.ROSDOC_DOT+"/io/%s.png"%node_name)
        
        p = HtmlElement(html.Grouping.p)
        p.set("align","center")
        img = HtmlElement(html.EmbeddedContent.img)
        img.set("src","../dot/io/%s.png"%node_name)
        
        p.append(img)
        self.append(p)
        
        return True
    
    def _read_topics(self, node, digraph):
        
        if node is None or node.tag != "topics":
            html.HTMLException("Invalid topics container !", self)
            return
        
        subs_dot_model = SubscribersModel()
        pubs_dot_model = PublishersModel()
        has_pub = False
        has_sub = False
        
        for child in node:
            if child.tag == "publisher":
                pubs_dot_model.addPublisher(child.attrib["name"],
                                            child.attrib["msg"])
                has_pub=True
            elif child.tag == "subscriber":
                subs_dot_model.addSubscriber(child.attrib["name"],
                                             child.attrib["msg"])
                has_sub=True
            else:
                print "Unknown topics tag '%s' !"%child.tag
        
        if has_pub is True:
            digraph.addNode(pubs_dot_model)
            digraph.connect(digraph.getRootNode(), pubs_dot_model)
        
        if has_sub is True:
            digraph.addNode(subs_dot_model)
            digraph.connect(subs_dot_model, digraph.getRootNode())
    
    def _read_actionlibs(self, node, digraph):
        
        if node is None or node.tag != "actionlib":
            html.HTMLException("Invalid actionlib container !", self)
            return
        
        for child in node:
            if child.tag == "client":
                try:
                    ns = ActionClientModel(child.attrib["name"], child.attrib["action"])
                    digraph.addNode(ns)
                    digraph.connect(digraph.getRootNode(), ns, Edge(Edge.DIR, "both"))
                except:
                    continue
            elif child.tag == "server":
                try:
                    nc = ActionServerModel(child.attrib["name"], child.attrib["action"])
                    digraph.addNode(nc)
                    digraph.connect(nc, digraph.getRootNode(), Edge(Edge.DIR, "both"))
                except:
                    continue
            else:
                print "Unknown actionlibs tag '%s' !"%child.tag
        
    def _read_services(self, node, digraph):
        
        if node is None or node.tag != "services":
            html.HTMLException("Invalid services container !", self)
            return
        
        for child in node:
            if child.tag == "client":
                print "Service client : ",child.attrib
            elif child.tag == "server":
                print "Service server : ",child.attrib
            else:
                print "Unknown services tag '%s' !"%child.tag
    
