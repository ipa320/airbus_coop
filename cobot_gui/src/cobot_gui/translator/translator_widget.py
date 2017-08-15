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

import rospy
import os

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

from cobot_gui.account import Privilege
from pyqt_agi_extend.QtAgiGui import QAgiPopup

from cobot_gui.res import R

class CountryWidget(QPushButton):
    
    def __init__(self, parent, country):
        """! The constructor."""
        QPushButton.__init__(self, parent)
        
        self._parent = parent
        self._country = country
        
        self.setMinimumSize(QSize(40, 40))
        self.setMaximumSize(QSize(60, 60))
        self.setStyleSheet(R.values.styles.transparent_background)
        
        self.setIcon(R.getIconById(country))
        self.setIconSize(QSize(self.width(),self.height()))
        
    def mousePressEvent(self, event):
        self._parent.setLanguageSelected(self._country)
        self._parent.close()
        
## @class ChooseCountryPopup
## @brief User accounts popup ui.
class ChooseCountryPopup(QAgiPopup):
    
    LNGS = ['en','fr','de','es']
    
    def __init__(self, parent):
        """! The constructor."""
        QAgiPopup.__init__(self, parent)
        
        self._parent = parent
        
        self.setRelativePosition(QAgiPopup.TopRight, QAgiPopup.BottomRight)
        
        # Extend the widget with all attributes and children from UI file
        loadUi(R.layouts.languages_popup, self)
        
        for lng in self.LNGS:
            self.country_layout.addWidget(CountryWidget(self, lng))
        
        self.adjustSize()
        
    def setLanguageSelected(self, lng):
        self._parent.getContext().switchLanguage(lng)
        
        
    def closeEvent(self, event):
        self._parent.refresh()

## @class TranslatorGadget
## @brief Translator interface by language chossen.
class TranslatorUi(QLabel):
    
    def __init__(self, context):
        """! The constructor."""
        
        QLabel.__init__(self)
        
        self._context = context
        self._context.addLanguageEventListner(self.onTranslate)
        
        self.setMinimumSize(QSize(40, 40))
        self.setMaximumSize(QSize(60, 60))
        
    def getContext(self):
        return self._context
        
    def resizeEvent(self,event):
        """! Redefine qt methode for resize widget.
        @param event: qt event.
        @type event: QEvent.
        """
        self.refresh()
        
    def refresh(self):
        
        lng = self.getContext().getLanguage()
        
        self.setPixmap(R.getPixmapById(lng).scaled(
                                            self.width(),
                                            self.height(),
                                            Qt.KeepAspectRatio,
                                            Qt.SmoothTransformation)
                       )
        
    def mousePressEvent(self, event):
        """! Redefine qt methode for resize widget.
        @param event: qt event.
        @type event: QEvent.
        """
        
        popup = ChooseCountryPopup(self)
        popup.show_()
        
    def onTranslate(self, lng):
        pass
    
    def onDestroy(self):
        pass

if __name__ == "__main__":
    
    import sys
    from cobot_gui.context import Context
    
    app = QApplication(sys.argv)
    main = QMainWindow()
    main.setCentralWidget(TranslatorUi(Context(main)))
    main.show()
    app.exec_()

