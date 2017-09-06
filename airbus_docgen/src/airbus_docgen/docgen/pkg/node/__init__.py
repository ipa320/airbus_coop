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
from airbus_docgen.common import html
from airbus_docgen.common.html import HtmlElement

from airbus_docgen.docgen.pkg.node.description import NodeDescription
from airbus_docgen.docgen.pkg.node.ios import NodeInputOutput
from airbus_docgen.docgen.pkg.node.params import NodeParameters


class RosNode(HtmlElement):
    
    def __init__(self):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"node"})
        
    def read(self, node_name, node_xml, isection, iarticle):
        
        item_index = 0
        
        try:
            node_desc = NodeDescription()
            if node_desc.read(node_name, node_xml) is True:
                item_index += 1
                title = HtmlElement(html.Sections.h4)
                title.text = "%i.%i.%i. Description"%(isection, iarticle, item_index)
                self.append(title)
                self.append(node_desc)
        except Exception as ex:
            html.HTMLException(ex,self)
            
        try:
            node_io = NodeInputOutput()
            if node_io.read(node_name, node_xml) is True:
                item_index += 1
                title = HtmlElement(html.Sections.h4)
                title.text = "%i.%i.%i. Input/Output"%(isection, iarticle, item_index)
                self.append(title)
                self.append(node_io)
        except Exception as ex:
            html.HTMLException(ex,self)
            
        try:
            node_params = NodeParameters()
            if node_params.read(node_name, node_xml) is True:
                item_index += 1
                title = HtmlElement(html.Sections.h4)
                title.text = "%i.%i.%i. Parameter(s)"%(isection, iarticle, item_index)
                self.append(title)
                self.append(node_params)
        except Exception as ex:
            html.HTMLException(ex,self)
        
        if item_index is 0:
            return False
        else:
            return True
    
