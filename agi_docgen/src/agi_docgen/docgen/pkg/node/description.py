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

class NodeDescription(HtmlElement):
    
    def __init__(self):
        HtmlElement.__init__(self, tag=html.Grouping.p)
        
    def read(self, node_name, node_xml):
        
        try:
            self.append(node_xml.find("description"))
        except:
            try:
                self.text = node_xml.find("description").text
#                 self.text.replace("$")
            except:
                return False
            
        return True
    
        