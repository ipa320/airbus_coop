#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : launch.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

from agi_docgen import env
from agi_docgen.common import html
from agi_docgen.common.html import HtmlElement
from agi_docgen.digraph.digraph import *
from xml.etree import ElementTree
from roslib.packages import get_pkg_dir

class Prefix:
    
    KEY_FIND = 'find'
    KEY_ARG = 'arg'
    
    @staticmethod
    def is_prefixed(path):
        if '$' in path:
            return True
        else:
            return False
    
    @staticmethod
    def get_prefix(path):
        
        if Prefix.is_prefixed(path):
            try:
                
                path_split = path.split(")")
                prefix = path_split[0].split("(")[-1]
                prefix_args = prefix.split(' ')
                key = prefix_args[0]
                value = prefix_args[-1]
                rex = path_split[-1]
                return key, value, rex
            except :
                raise Exception("Bad prefix !")
        else:
            raise Exception("No profixed path !")
        
    @staticmethod
    def find(path):
        
        if Prefix.is_prefixed(path):
            try:
                key, value, rex = Prefix.get_prefix(path)
                if key == Prefix.KEY_FIND:
                    return get_pkg_dir(value)+rex
                else:
                    raise Exception("Bad prefixed key '%s'"%key)
            except Exception as ex:
                raise ex
        else:
            return path
        
    @staticmethod
    def arg(exp, tree):
        
        if Prefix.is_prefixed(exp):
            try:
                key, value, rex = Prefix.get_prefix(exp)
                if key == Prefix.KEY_ARG:
                    node_arg = tree.find('%s[@name="%s"]'%(key,value))
                    df = node_arg.attrib["default"]
                    if Prefix.is_prefixed(df):
                        Prefix.arg(df, tree)
                    else:
                        return node_arg.attrib["default"]+rex
                else:
                    raise Exception("Bad prefixed key '%s'"%key)
            except Exception as ex:
                raise ex
        else:
            return exp

class ConfigLaunch(HtmlElement):
    
    def __init__(self):
        HtmlElement.__init__(self,
                             tag=html.Sections.article)
        
        
    def read(self, launch_file):
        
        digraph = Digraph("LaunchGraph")
        digraph.setAttrib(Digraph.NODESEP, 0.1)
#         digraph.setAttrib(Digraph.RANKDIR, 'LR')
         
        nconf = NODE("node")
        nconf.setAttrib(NODE.SHAPE, SHAPE.Plaintext)
        digraph.addNode(nconf)
        
        name = launch_file.split('/')[-1].split('.')[0]
        
        pkg = NODE(name)
        pkg.setAttrib(NODE.SHAPE, SHAPE.Ellipse)
        pkg.setAttrib(NODE.STYLE, STYLE.FILLED)
        pkg.setAttrib(NODE.COLOR, RgbColor.CornflowerBlue)
        pkg.setAttrib(NODE.FONTSIZE, 22)
        digraph.addRootNode(pkg)
        
        self._parse(launch_file, digraph)
        
        digraph.saveDot(env.ROSDOC_DOT+"/launch/%s.dot"%name)
        digraph.dotToPng(env.ROSDOC_DOT+"/launch/%s.png"%name)
        
        a = HtmlElement(html.Text.a)
        a.set(html.Attrib.href,"../dot/launch/%s.png"%name)
        a.set("target", "_blank")
        
        p = HtmlElement(html.Grouping.p)
        p.set("align","center")
        img = HtmlElement(html.EmbeddedContent.img)
        img.set("src","../dot/launch/%s.png"%name)
        
        p.append(img)
        a.append(p)
        self.append(a)
    
    def _parse(self, launch_file, digraph):
        
        launch_name = launch_file.split('/')[-1].split('.')[0]
        
        root = None
        try:
            root = ElementTree.parse(launch_file)
        except Exception as ex:
            html.HTMLException(ex, self)
            return
        
        tree = root.getroot()
        
        for elem in tree:
            #<include file="$(find pma_startup)/launch/pma1_driver.launch" />
            if elem.tag == 'include':
                
                include_split = elem.attrib["file"].split(")")
                prefix = include_split[0]
                pkg_name = prefix.split(" ")[-1]
                
                child = include_split[-1].split("/")[-1].split('.')[0]
                
                n1 = NODE(launch_name)
                n1.setAttrib(NODE.SHAPE, SHAPE.Box)
                n1.setAttrib(NODE.STYLE, STYLE.FILLED)
                n1.setAttrib(NODE.COLOR, RgbColor.LightGray)
                digraph.addNode(n1)
                
                n2 = NODE(child)
                n2.setAttrib(NODE.SHAPE, SHAPE.Box)
                n2.setAttrib(NODE.STYLE, STYLE.FILLED)
                n2.setAttrib(NODE.COLOR, RgbColor.LightGray)
                digraph.addNode(n2)
                
                digraph.connect(n1, n2)
                
                sub_launch = get_pkg_dir(pkg_name)+include_split[-1]
                
                self._parse(sub_launch, digraph)
                
            #<node name="map_server_high_res" pkg="map_server" type="map_server" args="$(arg map_file_high_res)" respawn="false">
            elif elem.tag == "node":
                 
                parent = launch_name
                child = Prefix.arg(elem.attrib["name"],tree)
                if parent == child:
                    child += "_node"
                     
                nd = NODE(child)
                nd.setAttrib(NODE.SHAPE, SHAPE.Ellipse)
                nd.setAttrib(NODE.STYLE, STYLE.FILLED)
                nd.setAttrib(NODE.COLOR, RgbColor.LightSeaGreen)
                digraph.addNode(nd)
                digraph.connect(parent, nd)
                
        
        
if __name__ == '__main__':
    
    cl = ConfigLaunch()
    
    