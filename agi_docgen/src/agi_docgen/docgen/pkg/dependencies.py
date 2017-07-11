#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : dependencies.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : dependencies.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
from agi_docgen.common import html
from agi_docgen.common.html import HtmlElement

class PackageDependencies(HtmlElement):
    
    def __init__(self, pkgdir, pkg_xml):
        HtmlElement.__init__(self,
                             tag=html.Sections.article,
                             attrib={"class":"pkg-dep"})
        
        self._dep_lists = []
        
        p = HtmlElement(html.Grouping.p)
        p.text = 'List of first order '
        dep_href = HtmlElement(html.Text.a)
        dep_href.set("href","http://wiki.ros.org/catkin/package.xml")
        dep_href.set("target", "_blank")
        dep_href.text = "package dependencies :"
        p.append(dep_href)
        
        ul = HtmlElement(html.Grouping.ul)
        
        for dep in pkg_xml.iter("build_depend"):
            li = HtmlElement(html.Grouping.li)
            li.text = dep.text
            self._dep_lists.append(dep.text)
            ul.append(li)
        
        p.append(ul)
        self.append(p)
        
    def get_dependencies_lists(self):
        return self._dep_lists
        