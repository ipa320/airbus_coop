#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : node.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import os
from agi_docgen.common import html
from agi_docgen.common.html import HtmlElement

from agi_docgen.docgen.pkg.node.description import NodeDescription
from agi_docgen.docgen.pkg.node.ios import NodeInputOutput
from agi_docgen.docgen.pkg.node.params import NodeParameters


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
    