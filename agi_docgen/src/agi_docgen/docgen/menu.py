#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : menu.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import os
from copy import copy
from copy import deepcopy
from agi_docgen import env
from agi_docgen.common import html
from agi_docgen.common.html import HtmlElement

class AbstractMenuItem(HtmlElement):
    
    def __init__(self, name, href='#', subitem=True):
        HtmlElement.__init__(self, tag = html.Grouping.li)
        
        root = HtmlElement(tag=html.Text.a)
        root.set(html.Attrib.href, href)
        root.text = name
        self.append(root)
        
        self._container_menu = HtmlElement(tag=html.Grouping.ul)
        
        if subitem:
            self.append(self._container_menu)
            
    def getRoot(self):
        return self._container_menu
        
    def addSubItem(self, parent, item_name):
        """
        Append element "<li><a>$menu_name</a><ul/></li>" into parent element
        """
        item = HtmlElement(tag=html.Grouping.li)
        name  = HtmlElement(tag=html.Text.a)
        name.text = item_name
        item.append(name)
        subitem = HtmlElement(tag=html.Grouping.ul)
        item.append(subitem)
        parent.append(item)
        
        return subitem
        
    def addItem(self, parent, item_name, href="#"):
        """
        Append element "<li><a href="$href">$pkg_name</a></li>" into <ul> parent stack element
        """
        li = HtmlElement(tag=html.Grouping.li)
        a = HtmlElement(tag=html.Text.a)
        a.set(html.Attrib.href, href)
        a.text = item_name
        li.append(a)
        parent.append(li)
        
        return li
        
    def __str__(self):
        html.indent(self)
        return html.tostring(self)

class AbstractMenu(HtmlElement):
    
    def __init__(self, menu_id):
        HtmlElement.__init__(self, tag=html.Grouping.div)
        self.set(html.Attrib.id, menu_id)
    
    def addItem(self, item):
        self.append(item)
    
    def __str__(self):
        html.indent(self)
        return html.tostring(self)
    
from agi_docgen.docgen.home import HtmlHomeFileGenerator

class HomeMenuItem(AbstractMenuItem):
    
    def __init__(self):
        AbstractMenuItem.__init__(self, "Home", "home.html", subitem=False)
        
    def generate_htmls(self, index, packages_dir=[]):
        home_docgen = HtmlHomeFileGenerator(deepcopy(index), packages_dir)
        home_docgen.save()

from agi_docgen.docgen.pkg import HtmlPkgFileGenerator

class PackagesMenuItem(AbstractMenuItem):
    
    def __init__(self):
        AbstractMenuItem.__init__(self, "Packages")
        self._base_dir = ""
        self._stack_register = {}
        self._xml_pkg_dir = []
        
    def get_packages_dir(self):
        return self._xml_pkg_dir
        
    def parse(self, workspace_dir):
        
        self._base_dir = copy(workspace_dir)
        self._parse(workspace_dir)
        
        return self
        
    def _parse(self, directory):
        
        stack = directory.split('/')[-1]
        
        for filename in os.listdir(directory):
            # If file is a stack -> iter
            if filename == stack:
                continue
            # If package -> end iteration
            elif filename == "package.xml":
                self._xml_pkg_dir.append(directory)
                rpath = directory.replace(self._base_dir+"/","")
                spath = rpath.split("/")
                self._gen_menu_(self.getRoot(), spath[:-1], spath[-1])
                continue
            # If available directory -> iter
            elif os.path.isdir(os.path.join(directory,filename)):
                self._parse(os.path.join(directory,filename))
            # Go to next list filename
            else:
                continue
            
    def _gen_menu_(self, parent, stacks, package):
        
        if len(stacks) > 0:
            substack = None
            stack = stacks[0]
            if stack not in self._stack_register.keys():
                substack = self.addSubItem(parent, stack)
                self._stack_register.update({stack : substack})
            else:
                substack = self._stack_register[stack]
            
            self._gen_menu_(parent=substack, stacks=stacks[1:], package=package)
        else:
            self.addItem(parent, package, "%s.html"%package)
        
    def generate_htmls(self, index):
        
        for pkg in self._xml_pkg_dir:
            # Provide package name
            pkg_name = pkg.split('/')[-1].replace(".xml","")
            print "Generate html file from package '%s' ..."%pkg_name
            pkg_docgen = HtmlPkgFileGenerator(deepcopy(index), pkg, pkg_name)
            pkg_docgen.save()

from agi_docgen.docgen.config import HtmlConfigFileGenerator

class ConfigMenuItem(AbstractMenuItem):
    
    def __init__(self):
        AbstractMenuItem.__init__(self, "Configurations", subitem=True)
        
        self._launch_file_dirs =[]
        
    def read(self, config_xml):
        
        for config in config_xml.iter("config"):
            
            config_src = config.attrib["src"].replace("${rossrc}", env.ROS_WS+"/src")
            config_name = config.attrib["name"]
            parent = self.addSubItem(self.getRoot(),config_name)
            
            for launch in config.iter("launch"):
                
                self._launch_file_dirs.append(os.path.join(config_src, launch.attrib['name']))
                
                launch_name = launch.attrib['name'].replace(".launch","")
                self.addItem(parent, launch_name, '%s.html'%launch_name)
        
    def generate_htmls(self, index):
        
        for ldir in self._launch_file_dirs:
            print "Generate html from launch '%s' ..."%ldir.split('/')[-1]
            launch_html = HtmlConfigFileGenerator(deepcopy(index), ldir)
            launch_html.save()
        

class Menu(AbstractMenu):
    
    def __init__(self):
        AbstractMenu.__init__(self, "mmenu")
        
        self._home_item = HomeMenuItem()
        self._pkgs_item = PackagesMenuItem()
        self._configs_item = ConfigMenuItem()
        
    def parse(self, wsdir):
        
        docgen_conf = html.loadHtml(os.path.join(env.ROSDOC_RSC,'docgen.conf'))
        menu_xml_conf = docgen_conf.find("menu")
        
        try:
            self._pkgs_item.parse(wsdir)
        except Exception as ex:
            raise ex
        
        try:
            self._configs_item.read(menu_xml_conf.find("configs"))
        except Exception as ex:
            raise ex
        
        self.addItem(self._home_item)
        self.addItem(self._pkgs_item)
        self.addItem(self._configs_item)
        
        self.addItem(AbstractMenuItem("Futures", subitem=False))
        self.addItem(AbstractMenuItem("About", subitem=False))
        self.addItem(AbstractMenuItem("Contact", subitem=False))
        
        return self
    
    def generate(self, index):
        self._home_item.generate_htmls(index, self._pkgs_item.get_packages_dir())
        self._pkgs_item.generate_htmls(index)
        self._configs_item.generate_htmls(index)

if __name__ == '__main__':
    menu = Menu()
    print str(menu.parse(None, os.path.join(os.getenv("ROS_WORKSPACE"),"src")))
    
    