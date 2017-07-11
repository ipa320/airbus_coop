#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : description.py
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

class ConfigDescription(HtmlElement):
    
    def __init__(self, pkgdir, pkg_xml):
        HtmlElement.__init__(self,
                             tag=html.Grouping.p)
        
        try:
            self.append(pkg_xml.find("./description"))
        except:
            try:
                self.text = pkg_xml.find("./description").text
            except:
                raise Exception("Connot found description tag into %s/package.xml"%pkgdir)
            
        