#!/usr/bin/env python
#
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


import subprocess
from xml.etree import ElementTree as ET
from agi_docgen.common.color import RgbColor
from agi_docgen.digraph.shape import SHAPE
from agi_docgen.digraph.arrow import ARROWHEAD
from agi_docgen.digraph.spline import SPLINE
from agi_docgen.digraph.style import STYLE
from agi_docgen.digraph.text import ALIGN

from agi_docgen.common import html

def indent(elem, level=0):
    
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

class TD(ET.Element):
    
    ALIGN="ALIGN"
    BALIGN="BALIGN"
    BGCOLOR="BGCOLOR"
    BORDER="BORDER"
    CELLPADDING="CELLPADDING"
    CELLSPACING="CELLSPACING"
    COLOR="COLOR"
    COLSPAN="COLSPAN"
    FIXEDSIZE="FIXEDSIZE"
    GRADIENTANGLE="GRADIENTANGLE"
    HEIGHT="HEIGHT"
    HREF="HREF"
    ID="ID"
    PORT="PORT"
    ROWSPAN="ROWSPAN"
    SIDES="SIDES"
    STYLE="STYLE"
    TARGET="TARGET"
    TITLE="TITLE"
    TOOLTIP="TOOLTIP"
    VALIGN="VALIGN"
    WIDTH="WIDTH"
    
    __ATTRIB_KEYS__ = [ALIGN,BALIGN,BGCOLOR,BORDER,CELLPADDING,CELLSPACING,
                       COLOR,COLSPAN,FIXEDSIZE,GRADIENTANGLE,HEIGHT,HREF,
                       ID,PORT,ROWSPAN,SIDES,STYLE,TARGET,TITLE,TOOLTIP,
                       VALIGN, WIDTH]
    
    def __init__(self):
        ET.Element.__init__(self, tag=html.Tables.td)
        
    def setAttrib(self, key, value):
        if key in self.__ATTRIB_KEYS__:
            self.set(key, str(value).replace("\"", ""))
        else:
            raise Exception("Invalid key attribute from '%s' !"%str(key))
        
    def setText(self, text):
        self.text = text
        
    def __str__(self):
        return ET.tostring(self, encoding="us-ascii", method="xml")

class TR(ET.Element):
    
    def __init__(self, td=None):
        ET.Element.__init__(self, tag=html.Tables.tr)# "TR")
        
        if td is not None:
            self.addTD(td)
    
    def addTD(self, td):
        self.append(td)
        
    def addTDs(self, tds):
        for td in tds:
            self.addTD(td)
        
    def __str__(self):
        return ET.tostring(self, encoding="us-ascii", method="xml")

class TABLE(ET.Element):
    
    ALIGN="ALIGN"
    BGCOLOR="BGCOLOR"
    BORDER="BORDER"
    CELLBORDER="CELLBORDER"
    CELLPADDING="CELLPADDING"
    CELLSPACING="CELLSPACING"
    COLOR="COLOR"
    COLUMNS="COLUMNS"
    FIXEDSIZE="FIXEDSIZE"
    GRADIENTANGLE="GRADIENTANGLE"
    HEIGHT="HEIGHT"
    HREF="HREF"
    ID="ID"
    PORT="PORT"
    ROWS="ROWS"
    SIDES="SIDES"
    STYLE="STYLE"
    TARGET="TARGET"
    TITLE="TITLE"
    TOOLTIP="TOOLTIP"
    VALIGN="VALIGN"
    WIDTH="WIDTH"
    
    __ATTRIB_KEYS__ = [ALIGN, BGCOLOR, BORDER, CELLBORDER, CELLPADDING,
                       CELLSPACING, COLOR, COLUMNS, FIXEDSIZE,
                       GRADIENTANGLE,  HEIGHT, HREF, ID, PORT, ROWS, SIDES,
                       STYLE, TARGET, TITLE, TOOLTIP, VALIGN, WIDTH]
    
    def __init__(self):
        ET.Element.__init__(self, tag=html.Tables.table)#"TABLE")
        
    def setAttrib(self, key, value):
        if key in self.__ATTRIB_KEYS__:
            self.set(key, str(value).replace("\"", ""))
        else:
            raise Exception("Invalid key attribute from '%s' !"%str(key))
    
    def addTR(self, tr):
        self.append(tr)
        
    def addTRs(self, tr_list):
        for tr in tr_list:
            self.addTR(tr)
        
    def __str__(self):
        indent(self)
        return ET.tostring(self, encoding="us-ascii", method="xml")
        
class NODE:
    
    MARGIN    = "margin" 
    FONTCOLOR = "fontcolor" 
    FONTSIZE  = "fontsize" 
    WIDTH     = "width" 
    SHAPE     = "shape" 
    STYLE     = "style"
    LABEL     = "label"
    DIR       = "dir"
    COLOR     = 'color'
    
    def __init__(self, node_name=""):
        self._node_name = NODE.formatNodeName(node_name)
        self._attribs = ""
        
    @staticmethod
    def formatNodeName(name):
        return name.replace('.', '_').replace('-','_').replace('/','_')
        
    def getName(self):
        return self._node_name
        
    def setAttrib(self, key, value):
        self._attribs += ' %s=%s'%(key, str(value))
    
    def setHtml(self, html):
        self._attribs += ' label=<%s>'%str(html)
        
    def setLabel(self, label):
        self._attribs += ' label="%s"'%str(label)
    
    def __str__(self):
        return "%s [%s];\n"%(self._node_name, self._attribs)
    
class Edge:
    
    DIR = "dir"
    
    def __init__(self, key=None, value=None):
        self._attrib = "["
        
        if key is not None and value is not None:
            self.setAttrib(key, value)
    
    def setAttrib(self, key, value):
        self._attrib += "%s=%s "%(key, str(value))
        
        return self
        
    def __str__(self):
        return "%s];\n"%self._attrib

class Digraph:
    
    ROOT = 'root'
    DIGRAPH  = "digraph"
    SUBGRAPH = "subgraph"
    STYLE = "style"
    COLOR = "color"
    RANKDIR = "rankdir"
    SPLINE = "splines"
    NODESEP = 'nodesep'
    
    def __init__(self, graph_name, graph_type=DIGRAPH):
        self._graph_name = graph_name
        self._graph_type = graph_type
        self._attribs = ""
        self._node_attribs = ""
        self._root_node = ""
        self._nodes = ""
        self._connections = ""
        self._dot_src_dir = None
        
    def getType(self):
        return self._graph_type
        
    def getName(self):
        return self._graph_name
        
    def setAttrib(self, key, value):
        self._attribs += '%s=%s;\n'%(key,str(value))
        
    def setLabel(self, label):
        self._attribs += 'label="%s";\n'%label
    
    def addNode(self, node):
        if isinstance(node, NODE):
            self._nodes += str(node)
        else:
            raise Exception("Bad class instance from NODE !")
        
    def addRootNode(self,node):
        self._root_node = node.getName()
        self.addNode(node)
        
    def getRootNode(self):
        return self._root_node
        
    def addSubGraph(self, graph):
        if graph.getType() == self.SUBGRAPH:
            self._nodes += str(graph)
        else:
            raise Exception("Bad class instance from SubGraph !")
    
    def _resolve_(self, obj):
        
        obj_str = ""
        
        if isinstance(obj, NODE) or isinstance(obj, Digraph):
            obj_str = obj.getName()
        elif isinstance(obj, dict):
            k = obj.keys()[0]
            v = obj.values()[0]
            obj_str = "%s:%s"%(str(k.getName()),v)
        else:
            obj_str = obj
            
        return obj_str
        
    def connect(self, node1, node2, edge_attrib=None):
        
        if node1 is None or node2 is None:
            return
        
        if edge_attrib is not None:
            self._connections += "%s -> %s %s"%(self._resolve_(node1),
                                                self._resolve_(node2),
                                                str(edge_attrib))
        else:
            self._connections +="%s -> %s;\n"%(self._resolve_(node1),
                                               self._resolve_(node2))
        
    def connectRank(self, key, nodes=[], edge=None):
        
        if nodes is None:
            return
        
        if len(nodes) >= 2:
            
            self._connections += "{rank=%s;"%key
            
            begin = " "
            for i in range(1, len(nodes)):
                self._connections += "%s%s"%(begin,self._resolve_(nodes[i-1]))
                self._connections += " -> %s"%self._resolve_(nodes[i])
                begin = " -> "
            
            self._connections += ";}\n"
        
    def __str__(self):
        
        if self._node_attribs != "":
            self._node_attribs = "node [%s]\n"%self._node_attribs
        
        graph_str = "%s %s {\n%s%s%s%s}\n"%(
                                self._graph_type,
                                self._graph_name,
                                self._attribs,
                                self._node_attribs,
                                self._nodes,
                                self._connections)
        
        return graph_str
    
    def saveDot(self, output):
        
        if '.dot' not in output:
            output+=".dot"
            
        self._dot_src_dir = output
        
        with open(self._dot_src_dir,'w') as f_dot:
            f_dot.write(self.__str__())
        
    
    def dotToPng(self, output, rm_dot=False):
        
        if '.png' not in output:
            output+=".png"
        
        if self._dot_src_dir is None:
            raise Exception("You must save dot before converting to png file !")
        
        cmd=["dot", "-Tpng", self._dot_src_dir, "-o", output]
        subprocess.Popen(cmd)
        
    def dotToPs(self, output, rm_dot=False):
        
        if '.ps' not in output:
            output+=".ps"
        
        if self._dot_src_dir is None:
            raise Exception("You must save dot before converting to ps file !")
        
        cmd=["dot", "-Tps", self._dot_src_dir, "-o", output]
        subprocess.Popen(cmd)
        

def digraph_test():
    
    digraph = Digraph("unittest")
    digraph.setAttrib('rankdir','LR')
    
    head = NODE("node")
    head.setAttrib(NODE.SHAPE, SHAPE.Plaintext)
    digraph.addNode(head)
    
    node1 = NODE("node_test")
    node1.setAttrib(NODE.SHAPE, SHAPE.Plaintext)
    node1.setAttrib(NODE.FONTCOLOR, RgbColor.Red)
    
    table = TABLE()
    table.setAttrib(TABLE.BORDER, 0)
    table.setAttrib(TABLE.CELLBORDER, 1)
    table.setAttrib(TABLE.CELLSPACING, 0)
    
    tr = TR()
    
    td = TD()
    td.setAttrib(TD.PORT, "input")
    td.setText("test")
    
    td2 = TD()
    td2.setAttrib(TD.PORT, "output")
    td2.setAttrib(TD.COLOR, RgbColor.Blue)
    td2.setText("test2")
    
    tr.addTD(td)
    tr.addTD(td2)
    
    table.addTR(tr)
    
    node1.setHtml(table)
    
    node2 = NODE("node_test_2")
    #{
    table = TABLE()
    table.setAttrib(TABLE.BORDER, 0)
    table.setAttrib(TABLE.CELLBORDER, 10)
    
    tr = TR()
    td = TD()
    td.setAttrib(TD.PORT, "input")
    td.setText("test")
    td2 = TD()
    td2.setAttrib(TD.PORT, "output")
    td2.setAttrib(TD.COLOR, RgbColor.Red)
    td2.setText("test3")
    
    tr.addTD(td)
    tr.addTD(td2)
    
    table.addTR(tr)
    
    node2.setHtml(table)
    
    node3 = NODE("test11")
    node3.setLabel("Node for testing none html label !")
    node3.setAttrib(NODE.SHAPE, SHAPE.Ellipse)
    
    digraph.addNode(node1)
    digraph.addNode(node2)
    digraph.addNode(node3)
    
#     digraph.connect({node1:"output"}, {node2:"input"})
#     digraph.connect({node2:"output"}, {node1:"input"})
#     digraph.connect(node3, node1)
    digraph.connectRank("same", [node1, node2, node3])
    
    print str(digraph)
    
    digraph.saveDot("/tmp/digraph_test.dot")
    digraph.dotToPng("/tmp/digraph_test.png")

def getSubGraph(name):
    
    subgraph = Digraph(name, Digraph.SUBGRAPH)
    subgraph.setLabel(name)
    subgraph.setAttrib(Digraph.STYLE, "filled")
    subgraph.setAttrib(Digraph.COLOR, RgbColor.Cyan)
    
    node1 = NODE("node1_%s"%name)
    node1.setAttrib(NODE.SHAPE, SHAPE.Diamond)
    subgraph.addNode(node1)
    
    node2 = NODE("node2_%s"%name)
    node2.setAttrib(NODE.SHAPE, SHAPE.House)
    subgraph.addNode(node2)
    
    subgraph.connect(node1, node2)
    
    return subgraph
    
if __name__ == '__main__':
    
    digraph_test()
    
#     graph = Digraph("root_graph")
#     graph.setLabel("Root Graph")
#     
#     root = NODE("root")
#     root.setLabel("Root into root_graph !")
#     root.setAttrib(NODE.SHAPE, SHAPE.Doubleoctagon)
#     root.setAttrib(NODE.FONTCOLOR, RGB.Red)
#     graph.addNode(root)
#     
#     sub1 = getSubGraph("sub1")
#     sub2 = getSubGraph("sub2")
#     
#     graph.addSubGraph(sub1)
#     graph.addSubGraph(sub2)
#     
#     graph.connect(root , "node1_sub1")
#     graph.connect(root , "node1_sub2")
#     
#     print str(graph)
#     
#     graph.saveDot("/tmp/subgraph_test.dot")
#     graph.dotToPng("/tmp/subgraph_test.png")