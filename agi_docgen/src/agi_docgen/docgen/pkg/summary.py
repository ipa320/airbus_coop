#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : summary.py
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

class PackageSummary(HtmlElement):
    
    def __init__(self, pkgdir, pkg_xml):
        HtmlElement.__init__(self,
                             tag=html.Grouping.ul)
        
        version = HtmlElement(html.Grouping.li)
        version.text = "Version : %s"%pkg_xml.find("./version").text
        self.append(version)
        
        mtr = pkg_xml.find("./maintainer")
        
        maintainer_li = HtmlElement(html.Grouping.li)
        maintainer_li.text = "Maintainer : "
        maintainer = HtmlElement(html.Text.a)
        try:
            maintainer.set(html.Attrib.href, "mailto:%s"%mtr.attrib['email'])
        except:
            pass
        maintainer.text = mtr.text
        maintainer_li.append(maintainer)
        self.append(maintainer_li)
        
        llicense = HtmlElement(html.Grouping.li)
        llicense.text = "License : %s"%pkg_xml.find("./license").text
        self.append(llicense)
        
        if pkg_xml.find("./url") is not None:
            li = HtmlElement(html.Grouping.li)
            li.text = "Link : "
            url = HtmlElement(html.Text.a)
            url.set("href",pkg_xml.find("./url").text)
            url.set("target", "_blank")
            url.text  = pkg_xml.find("./url").text
            li.append(url)
            self.append(li)
            
        if pkg_xml.find("./export/rosdoc") is not None:
            li = HtmlElement(html.Grouping.li)
            li.text = "Sources : "
            doxygen = HtmlElement(html.Text.a)
            ref = pkgdir+"/doc/html/index.html"
            doxygen.set("href", ref)
            doxygen.set("target", "_blank")
            doxygen.text = "doxygen"
            li.append(doxygen)
            self.append(li)
            