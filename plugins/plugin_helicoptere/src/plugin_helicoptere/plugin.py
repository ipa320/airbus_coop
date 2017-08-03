#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : plugin.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
import rospy
import os
import math

from std_msgs.msg import String

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from pyqt_agi_extend.QtAgiGui import QAgiMessageBox

from cobot_gui import Plugin, ControlMode

from plugin_helicoptere.res import R

class SelectableImage(QLabel):
    
    def __init__(self, image_name):
        QLabel.__init__(self)
        self.setAlignment(Qt.AlignHCenter)
        
        self._image_name = image_name
        self._image = R.getPixmapById(image_name)
        
        self._selected = False
        
        self.setStyleSheet("background-color: rgba(255,255,255, 100);");
        
    def name(self):
        return self._image_name
        
    def isSelected(self):
        return self._selected
    
    def unselect(self):
        self._selected = False
        self.setStyleSheet("background-color: rgba(255,255,255, 100);");
        
    def resizeEvent(self, event):
        self.setPixmap(self._image.scaled(
                       self.width()-5,
                       self.height()-5,
                       Qt.KeepAspectRatio,
                       Qt.SmoothTransformation))
        
    def mousePressEvent(self, event):
        
        if self._selected is False:
            self._selected = True
            self.setStyleSheet("background-color: rgba(0,255,0,50);");
        else:
            self._selected = False
            self.setStyleSheet("background-color: rgba(255,255,255, 100);");
            
        self.emit(SIGNAL("imageTriggered"), self)

class HelicopterePlugin(Plugin):
    
    NB_IMAGES  = 8
    MAX_IMAGES_CAN_SELECT = 3
    TOPIC_NAME = "/helicoptere/trigger"
    
    def __init__(self, context):
        Plugin.__init__(self, context)
        
    def onCreate(self, param):
        
        loadUi(R.layouts.mainwidow, self)
        
        lng = self.getContext().getLanguage()
        self.title.setText(R.values.strings.title(lng))
        self.validate_button.setText(R.values.strings.validate(lng))
        
        self._images_publish = rospy.Publisher(self.TOPIC_NAME, String, queue_size=1)
        
        self.connect(self.validate_button, SIGNAL("clicked()"), self.onPublishImagesSelected)
        
        self._nb_images_selected = 0
        self._images=[]
        
        for i in range(1, self.NB_IMAGES+1):
            selectable = SelectableImage("image_%i"%i)
            self._images.append(selectable)
            self.connect(selectable, SIGNAL("imageTriggered"), self.imageTriggered)
        
        self.images_grid.addWidget(self._images[0], 0, 1)
        self.images_grid.addWidget(self._images[1], 0, 2)
        self.images_grid.addWidget(self._images[2], 0, 3)
        
        self.images_grid.addWidget(self._images[3], 1, 1)
        self.images_grid.addWidget(self._images[4], 1, 2)
        self.images_grid.addWidget(self._images[5], 1, 3)
        
        self.images_grid.addWidget(self._images[6], 2, 1)
        self.images_grid.addWidget(self._images[7], 2, 2)
        
    def imageTriggered(self, selectable_object):
        
        if selectable_object.isSelected() is True:
            self._nb_images_selected+=1
        else:
            self._nb_images_selected-=1
            
        if self._nb_images_selected > self.MAX_IMAGES_CAN_SELECT:
            lng = self.getContext().getLanguage()
            mbox = QAgiMessageBox(QAgiMessageBox.WARN,
                                  R.values.strings.max_images_reached(lng))
            mbox.exec_()
            selectable_object.unselect()
            self._nb_images_selected-=1
            
    def onPublishImagesSelected(self):
        
        if self._nb_images_selected > 0:
            
            msg = String()
            
            for img in self._images:
                if img.isSelected() is True:
                    msg.data += "/%s"%img.name()
            self._images_publish.publish(msg)
            
        else:
            lng = self.getContext().getLanguage()
            mbox = QAgiMessageBox(QAgiMessageBox.WARN,
                                  R.values.strings.no_images_selected(lng))
            mbox.exec_()
        
    def onStart(self):
        self.onResume()
        
    def onPause(self):
        pass
        
    def onResume(self):
        pass
        
    def onControlModeChanged(self, mode):
        
        if mode == ControlMode.AUTOMATIC:
            self.setEnabled(False)
        else:
            self.setEnabled(True)
        
    def onUserChanged(self, user_info):
        pass
    
    def onTranslate(self, lng):
        self.title.setText(R.values.strings.title(lng))
        self.validate_button.setText(R.values.strings.validate(lng))
        
    def onEmergencyStop(self, state):
        pass
        
    def onDestroy(self):
        pass
    
    # Resize background
    def resizeEvent(self, event):
        pass
    
#End of file

