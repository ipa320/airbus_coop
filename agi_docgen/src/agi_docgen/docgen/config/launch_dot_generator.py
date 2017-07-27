#!/usr/bin/env python
#-----------------------------------------------------------------------
# Description
#-----------------------------------------------------------------------
# Name:        File_list
# Purpose:
#
# Author:      Martin Matignon
#-----------------------------------------------------------------------
# History
#-----------------------------------------------------------------------
#
# 01/12/2015 Created ................................... Martin MATIGNON
#-----------------------------------------------------------------------
# Copyright:   (c) Martin Matignon
# Licence:     <your licence>
#-----------------------------------------------------------------------

import os
import sys
import time
import subprocess

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

class RosLaunchDotGenerator:
    
    def __init__(self):
        
        self._launch_directory=""
        self._launch_list=[]
        self._dot_filename=""
        
    def _parse(self, launch, dot):
        
        self._launch_directory = launch
        launch_name = launch.split('/')[-1].split('.')[0]
        
        root = None
        try:
            root = ElementTree.parse(self._launch_directory)
        except:
            raise Exception("Invalid launcher path from %s !"%self._launch_directory)
        
        tree = root.getroot()
        
        for elem in tree:
            #<include file="$(find pma_startup)/launch/pma1_driver.launch" />
            if elem.tag == 'include':
                
                include_split = elem.attrib["file"].split(")")
                prefix = include_split[0]
                pkg_name = prefix.split(" ")[-1]
                
                child = include_split[-1].split("/")[-1].split('.')[0]
                
                dot.write('%s [shape=box, style=filled, color=lightgray];\n'%launch_name)
                dot.write('%s [shape=box, style=filled, color=lightgray];\n'%child)
                dot.write("%s -> %s\n"%(launch_name, child))
                
                sub_launch = get_pkg_dir(pkg_name)+include_split[-1]
                
                self._parse(sub_launch, dot)
                
            #<node name="map_server_high_res" pkg="map_server" type="map_server" args="$(arg map_file_high_res)" respawn="false">
            elif elem.tag == "node":
                
                parent = launch_name
                child = Prefix.arg(elem.attrib["name"],tree)
                if parent == child:
                    child += "_node"
                
                dot.write('%s [shape=ellipse, style=filled, color=lightseagreen];\n'%child)
                dot.write("%s -> %s\n"%(parent,child))
                
    def parse(self, launch, dot_filename):
        
        self._dot_filename=dot_filename
        
        dot_file = open(dot_filename,'w')
        dot_file.write("digraph LaunchDot {\n")
        
        self._parse(launch, dot_file)
        
        dot_file.write("}")
        dot_file.close()
        
    def to_ps(self, ps_filename):
        cmd=["dot", "-Tps", self._dot_filename, "-o", "%s.ps"%ps_filename]
        subprocess.Popen(cmd)
        
#     def to_png(self, ps_filename):
#         cmd=["dot", "-Tpng", "%s.dot"%self._dot_filename, "-o", "%s.png"%ps_filename]
#         subprocess.Popen(cmd)
        

if __name__ == '__main__':
    
    dep = RosLaunchDotGenerator()
