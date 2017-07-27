#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : build.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################

from agi_docgen.digraph.digraph import *

def getTopicTRModel(topic, msg, bgcolor=RgbColor.White, align = ALIGN.Left):
    
    model = TR()
    
    topic_td = TD()
    topic_td.setAttrib(TD.COLSPAN, 1)
    topic_td.setAttrib(TD.ALIGN, align)
    topic_td.setAttrib(TD.BGCOLOR, bgcolor)
    topic_td.setText(topic)
    model.addTD(topic_td)
    
    msg_td = TD()
    msg_td.setAttrib(TD.COLSPAN, 1)
    msg_td.setAttrib(TD.ALIGN, align)
    msg_td.setAttrib(TD.BGCOLOR, bgcolor)
    msg_td.setText(msg)
    model.addTD(msg_td)
    
    return model

def getStandardTDModel(text, bgcolor=RgbColor.White, align = ALIGN.Left):
    
    model = TD()
    model.setAttrib(TD.COLSPAN, 1)
    model.setAttrib(TD.ALIGN, align)
    model.setAttrib(TD.BGCOLOR, bgcolor)
    model.setText(text)
    
    return model

class _IoTopicsModel(NODE):
    
    def __init__(self, node_name, node_title):
        NODE.__init__(self, node_name)
        
        self._table = TABLE()
        self._table.setAttrib(TABLE.BORDER, 0)
        self._table.setAttrib(TABLE.CELLBORDER, 1)
        self._table.setAttrib(TABLE.CELLSPACING, 0)
        self._table.setAttrib(TABLE.BGCOLOR, RgbColor.White)
        
        title = TD()
        title.setAttrib(TD.ALIGN, ALIGN.Center)
        title.setAttrib(TD.BGCOLOR, RgbColor.LightSkyBlue)
        title.setAttrib(TD.COLSPAN, 2)
        title.setText(node_title)
        self._table.addTR(TR(title))
        
        header = getTopicTRModel("Topic name", "Message type", bgcolor=RgbColor.LightGray, align=ALIGN.Center)
        self._table.addTR(header)
        
    def addSubscriber(self, topic_name, msg):
        sub = getTopicTRModel(topic_name, msg)
        self._table.addTR(sub)
        
    def addPublisher(self, topic_name, msg):
        self.addSubscriber(topic_name, msg)
        
    def getNode(self):
        self.setHtml(self._table)
        return self
    
    def __str__(self):
        self.setHtml(self._table)
        return NODE.__str__(self)
    
class SubscribersModel(_IoTopicsModel):
    def __init__(self):
        _IoTopicsModel.__init__(self, 'subscribers', "Subscribers list")
        
class PublishersModel(_IoTopicsModel):
    def __init__(self):
        _IoTopicsModel.__init__(self, 'publishers', "Publishers list")
        
# TFs Model

if __name__ == '__main__':

    digraph = Digraph("digraph_test")
    digraph.setAttrib(Digraph.RANKDIR,'LR')
    
    nconf = NODE("node")
    nconf.setAttrib(NODE.SHAPE, SHAPE.Plaintext)
    
    subs = SubscribersModel()
    
    subs.addSubscriber("/vel1","geometry_msgs::Twist1")
    subs.addSubscriber("/vel2","geometry_msgs::Twist2")
    subs.addSubscriber("/vel3","geometry_msgs::Twist3")
    subs.addSubscriber("/vel4","geometry_msgs::Twist4")
    
    digraph.addNode(subs)
    
    pubs = PublishersModel()
    
    pubs.addSubscriber("/vel1","geometry_msgs::Twist1")
    pubs.addSubscriber("/vel2","geometry_msgs::Twist2")
    pubs.addSubscriber("/vel3","geometry_msgs::Twist3")
    pubs.addSubscriber("/vel4","geometry_msgs::Twist4")
    
    digraph.addNode(pubs)
    
    digraph.connect(subs,pubs)
    
    #ToDo: get the path from user input (by default save into /tmp folder) or use rospack.get_path(agi_docgen)...
    digraph.saveDot("/home/ihm-pma/Documents/dot_test/SubscribersModel.dot") 
    digraph.dotToPng("/home/ihm-pma/Documents/dot_test/SubscribersModel.png")

    
    
