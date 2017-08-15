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


import sys

import rospy
import os

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi
from roslib.packages import get_pkg_dir
from xml.etree import ElementTree

from packages import get_pkg_dir_from_prefix, \
                     get_ros_workspace_dir, \
                     get_ros_workspace_src_dir

class _Image:
    
    def __init__(self, node, rsc_path):
        
        self.label   = ""
        self.src     = ""
        self.width   = 0
        self.height  = 0
        self.scale   = True
        self.preload = False
        self.pixmap  = None
        
        try:
            self._read_img_node(node)
        except Exception as ex:
            raise Exception(ex)
        
        self.src = os.path.join(rsc_path, self.src)
        
        if self.preload:
            self.pixmap = _Image.loadPixmap(self)
        
        
    def _read_img_node(self, node):
        
        if len(node.attrib.keys()) != 6:
            raise Exception('<img .../> takes exactly 6 arguments (%i given) !'%len(node.attrib.keys()))
        
        for k, v in node.attrib.items():
            
            if k == 'label':
                self.label = v
            elif  k == 'src':
                self.src = v
            elif  k == 'width':
                self.width = int(v)
            elif  k == 'height':
                self.height = int(v)
            elif  k == 'auto-scale':
                if v.lower() == 'false':
                    self.scale = False 
                else:
                    self.scale = True
            elif  k == 'preload':
                if v.lower() == 'true':
                    self.preload = True 
                else:
                    self.preload = False
            else:
                raise Exception('Inconstante parameter type <img %s="..."/>!'%k)
            
    def isPreloaded(self):
        return self.preload
    
    @staticmethod
    def loadPixmap(img_desc):
        
        try:
            px = QPixmap(img_desc.src)
            
            if img_desc.scale is True :
                px = px.scaled(img_desc.width, img_desc.height,
                               Qt.KeepAspectRatio,
                               Qt.SmoothTransformation)
            return px
        except Exception as ex:
            raise Exception(ex)
    
    def getPixmap(self, w = None, h = None):
        
        rescale = False
        if w is not None and h is not None:
            if w != self.width or h != self.height:
                self.width  = w
                self.height = h
                rescale = True
        
        if self.isPreloaded():
            if rescale:
                self.pixmap = _Image.loadPixmap(self)
            return self.pixmap
        else:
            return _Image.loadPixmap(self)
        
class QAgiImageRessource:
    
    def __init__(self, xnode):
        
        if xnode is None or 'path' not in xnode.attrib.keys():
            return
        
        self._img_rsc = {}
        ipath = get_pkg_dir_from_prefix(xnode.attrib['path'])
        
        if not os.path.isdir(ipath):
            raise Exception("Invalid path from images ressource !")
        
        for img_node in xnode:
            img = _Image(img_node, ipath)
            self._img_rsc.update({img.label : img})
    
    def getPixmap(self, key, w=None, h=None):
        
        try:
            img = self._img_rsc[key]
            return img.getPixmap(w,h)
        except Exception as ex:
            raise Exception(str(ex))
        
#     def getPixmap(self, key, size):
#         self.getPixmap(key, size.width(), size.height())
        
    def getPixmapSize(self, key):
        try:
            px = self._img_rsc[key]
            return QSize(px.width, px.height)
        except Exception as ex:
            raise Exception(str(ex))
        
    def getIcon(self, key):
        try:
            ico = self._img_rsc[key]
            return QIcon(ico.src)
        except Exception as ex:
            raise Exception(str(ex))
    
    def getIconSize(self, key):
        try:
            ico = self._img_rsc[key]
            return QSize(ico.width, ico.height)
        except Exception as ex:
            raise Exception(str(ex))
        
class QAgiUiRessource:
    def __init__(self, xnode):
        
        if xnode is None or 'path' not in xnode.attrib.keys():
            return
        
        self._uis_rsc = {}
        gpath = get_pkg_dir_from_prefix(xnode.attrib['path'])
        
        if not os.path.isdir(gpath):
            raise Exception("Invalid path from ui ressource !")
        
        for ui in xnode:
            uipath = '%s/%s'%(gpath, ui.attrib['src'])
            self._uis_rsc.update({ui.attrib['label']: uipath})
        
    def __getitem__(self, key):
        try:
            return self._uis_rsc[key]
        except Exception as ex:
            raise Exception(ex)
        
    def load(self, key, widget):
        try:
            loadUi(self.__getitem__(key), widget)
        except Exception as ex:
            raise ex
        
class QAgiYamlRessource:
    def __init__(self, xnode):
        
        if xnode is None or 'path' not in xnode.attrib.keys():
            return
        
        self._yamls_rsc = {}
        gpath = get_pkg_dir_from_prefix(xnode.attrib['path'])
        
        if not os.path.isdir(gpath):
            raise Exception("Invalid path from yamls ressource !")
        
        for yaml in xnode:
            yamlpath = '%s/%s'%(gpath, yaml.attrib['src'])
            self._yamls_rsc.update({yaml.attrib['label']: yamlpath})
        
    def __getitem__(self, key):
        try:
            return self._yamls_rsc[key]
        except Exception as ex:
            raise Exception(ex)

class QAgiStyleSheetRessource:
    
    def __init__(self, xnode):
        
        self._node = xnode
        self._theme = "default"
        
        if "theme" in self._node.attrib.keys():
            self._theme = self._node.attrib["theme"]
    
    def __getitem__(self, key):
        
        xpath = ""
        stylesheet = ""
        
        if '.' in key:
            keys = key.split('.')
            xpath = './sheet[@name="%s"]'%keys[0]
            for i in range(1, len(keys)):
                xpath+='/sheet[@name="%s"]'%keys[i]
        else:
            xpath = './sheet[@name="%s"]'%key
            
        xpath += '/%s'%self._theme
        
#         print xpath
        
        try:
            sheet = self._node.find(xpath)
            stylesheet = sheet.text
        except:
            rospy.logerr('Style sheet "%s" was not found !'%key)
        
        return stylesheet
    
class QAgiTraductionRessource:
    
    def __init__(self, xnode):
        
        self._node = xnode
        self._lng = "en"
        
        if "lng" in self._node.attrib.keys():
            self._lng = self._node.attrib["lng"]
    
    def __getitem__(self, key):
        
        trad  = str(key)
        xpath = './translate[@src="%s"]/%s'%(str(key), self._lng)
        
        try:
            trad = self._node.find(xpath).text
        except:
            rospy.logerr('Traduction from "%s" not found !'%key)
        
        return trad

class QAgiResources:
    
    DEFAULT_RSC_NAMES = ["rsc","resource","resources"]
    
    def __init__(self, package, dir):
        
        tree = None
        root = None
        rsc  = None
        
        if dir is None:
            rsc = self.find_default_rsc_file(package)
            if rsc is None:
                raise Exception("Connot find ressources file in pakage '%s'"%package)
        else:
            rsc = os.path.join(get_pkg_dir(package), dir)
        
        try:
            tree = ElementTree.parse(rsc)
            root = tree.getroot()
        except Exception as e:
            raise Exception(e)
        
        for node in root:
            if node.tag == "uis":
                setattr(self, node.tag, QAgiUiRessource(node))
            elif node.tag == "images":
                setattr(self, node.tag, QAgiImageRessource(node))
            elif node.tag == "yamls":
                setattr(self, node.tag, QAgiYamlRessource(node))
            elif node.tag == "traductions":
                setattr(self, node.tag, QAgiTraductionRessource(node))
            elif node.tag == "styles":
                setattr(self, node.tag, QAgiStyleSheetRessource(node))
            
        
    def find_default_rsc_file(self, pkg):
        
        pkg_path = get_pkg_dir(pkg)
        
        for name in self.DEFAULT_RSC_NAMES:
            dir = os.path.join(pkg_path, name)
            if os.path.isdir(dir):
                return os.path.join(dir,self.find_rsc_xml(dir))
        return None
    
    def find_rsc_xml(self, rsc_dir):
        
        for file in os.listdir(rsc_dir):
            file_split = file.split(".")
            if file_split[0] in self.DEFAULT_RSC_NAMES and file_split[-1] == "xml":
                return file
        return None
    
def loadRsc(package, dir=None):
    return QAgiResources(package, dir)

import copy

def loadRes(package):
    
    from packages import QAgiPackages
    
    res_class_name = ''
    
    for subs in package.split('_'):
        res_class_name+=subs[0].upper()+subs[1:]
    res_class_name+="Res"
    
    print 'Loading ressource name : %s'%res_class_name
    
    return QAgiPackages.__pkg__(package).__module__('resources.res').__import__(res_class_name)
    