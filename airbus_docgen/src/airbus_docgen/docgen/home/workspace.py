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
from airbus_docgen.common.html import HtmlElement
from airbus_docgen.digraph.digraph import *
import catkin_pkg.workspaces
import os

class RosWorkspace(HtmlElement):
    
    def __init__(self, packages_dir_list):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"ros_ws"})
        
        self.generate_ws_dot(packages_dir_list)
        
        a = HtmlElement(html.Text.a)
        a.set(html.Attrib.href,"../dot/agi_workspace.png")
        a.set("target", "_blank")
        
        p = HtmlElement(html.Grouping.p)
        p.set("align","center")
        
        img = HtmlElement(html.EmbeddedContent.img)
        img.set("src","../dot/agi_workspace.png")
        img.set("width","1024")
        img.set("height","150")
        
        p.append(img)
        
        a.append(p)
        
        self.append(a)
        
    def generate_ws_dot(self, packages_dir):
        
        digraph = Digraph("AgiWorksapce")
        digraph.setAttrib(Digraph.NODESEP, 0.1)
        
        nconf = NODE("node")
        nconf.setAttrib(NODE.SHAPE, SHAPE.Plaintext)
        digraph.addNode(nconf)
        
        pkg = NODE("ROS_WORKSPACE")
        pkg.setAttrib(NODE.SHAPE, SHAPE.Box)
        pkg.setAttrib(NODE.STYLE, STYLE.FILLED)
        pkg.setAttrib(NODE.COLOR, RgbColor.CornflowerBlue)
        digraph.addRootNode(pkg)
        
        connection_register = []
        
        for directory in packages_dir:
            rpkg = directory
            path = rpkg.split('/')
            
            for i in range(1,len(path)-1):
                
                conn = "%s->%s"%(path[i], path[i+1])
                
                if conn not in connection_register and path[i] != path[i+1]:
                    n1 = NODE(path[i])
                    n1.setAttrib(NODE.SHAPE, SHAPE.Box)
                    n1.setAttrib(NODE.STYLE, STYLE.FILLED)
                    n1.setAttrib(NODE.COLOR, RgbColor.LightGray)
                    
                    n2 = NODE(path[i+1])
                    if path[i+1] == path[-1]:
                        n2.setAttrib(NODE.SHAPE, SHAPE.Ellipse)
                        n2.setAttrib(NODE.STYLE, STYLE.FILLED)
                        n2.setAttrib(NODE.COLOR, RgbColor.LightSeaGreen)
                    else:
                        n2.setAttrib(NODE.SHAPE, SHAPE.Box)
                        n2.setAttrib(NODE.STYLE, STYLE.FILLED)
                        n2.setAttrib(NODE.COLOR, RgbColor.LightGray)
                    
                    digraph.addNode(n1)
                    digraph.addNode(n2)
                    digraph.connect(n1,n2)
                    connection_register.append(conn)
                    
        digraph.connect(digraph.getRootNode(), "src")
        
        digraph.saveDot(env.ROSDOC_DOT+"/agi_workspace.dot")
        digraph.dotToPng(env.ROSDOC_DOT+"/agi_workspace.png")
        
        
