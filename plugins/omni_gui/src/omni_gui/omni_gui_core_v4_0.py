################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : omni_gui_core_v4_0.py
# Authors : Clement Beaudoing
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Clement Beaudoing <clement.beaudoing.external@airbus.com>
#
#
################################################################################

"""
Graphical User Interface node core
"""
import os.path
import time, sys
from math import sqrt

from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *

import rospy
import roslib; roslib.load_manifest('omni_gui')
from geometry_msgs.msg import Twist

ACTIVE = 0
PENDING = 1
PAUSE = 2

## Max linear speed allowed (m/s)
MAX_LIN_SPEED = 0.600

## Max linear speed allowed (rad/s)
MAX_ANG_SPEED = 0.500

## @class ThreadPublisher
##
## @version 4.0
## @author Clement Beaudoing
## @date Last modification le 16/04/2014
class ThreadPublisher(QThread):
    def __init__(self):
        QThread.__init__(self)
        self._status = PENDING
        self._is_running = True
        
    def run(self):
        while(self._is_running):
            if not self._status == PENDING :
                try:
                    self.emit(SIGNAL('cmd_vel'))
                except:
                    pass
            #self.pub.publish(self._twist)
                if self._status == PAUSE:
                    self._status = PENDING
            time.sleep(0.01)
            
    def stop(self):
        self._is_running = False
    
    def thread_status(self, status):
        self._status = status

## @class CustomCentralScreen
##
## @version 4.0
## @author Clement Beaudoing
## @date Last modification le 16/04/2014
class CustomCentralScreen(QPushButton):
    ## @brief CustomCentralScreen constructor
    ##
    ## @version 4.0
    ## @author Clement Beaudoing
    ## @date Last modification le 16/04/2014
    def __init__(self, parent=None):
        super(CustomCentralScreen, self).__init__(parent)
        package_directory = roslib.packages.get_pkg_dir('omni_gui')
        self._twist = Twist() ## @var twist
        
        size_policy = QSizePolicy(QSizePolicy.Preferred,
                                  QSizePolicy.Preferred)
        size_policy.setHorizontalStretch(0)
        size_policy.setVerticalStretch(0)
        size_policy.setHeightForWidth(self.sizePolicy().hasHeightForWidth())
        self.setSizePolicy(size_policy)
        ## @var _my_pixmap 
        self._my_pixmap = QPixmap(package_directory +
                                  "/icons/cercle800segway_V3.2.png")
        self._my_pixmap.scaled(QSize(self._my_pixmap.width(),
                                     self._my_pixmap.height()),
                               Qt.KeepAspectRatio)
        self._my_icon = QIcon() ## @var my_icon
        self._my_icon.addPixmap(self._my_pixmap , QIcon.Normal, QIcon.Off)
        self.setIcon(self._my_icon)
        self.setFlat(True)
        self.setFocusPolicy(Qt.NoFocus)
        self.setCheckable(False)
        self.setAutoRepeat(True)
        self.setMinimumSize(100, 100)
        
        ## @var button_clicked
        self._lin_spe_button = QPushButton(self) 
        ## _lin_spe_button is set to the cursor position,
        ## when lilnear velocity command is requesting 
        self._lin_spe_button.setSizePolicy(size_policy)
        self._lin_spe_button._pixmap = QPixmap(package_directory +
                                               "/icons/led/blue-on-128.png")
        self._lin_spe_button._icon = QIcon()
        self._lin_spe_button._icon.addPixmap(self._lin_spe_button._pixmap,
                                                    QIcon.Normal,
                                                    QIcon.Off)
        self._lin_spe_button.setIcon(self._lin_spe_button._icon)
        self._lin_spe_button.setIconSize(QSize(75, 75))
        self._lin_spe_button.setGeometry(QRect(0, 0, 75, 75))
        self._lin_spe_button.setFlat(True)
        self._lin_spe_button.setCheckable(False)
        self._lin_spe_button.hide()
        self._lin_spe_button.state = False
        
        self._ang_spe_button = QPushButton(self)## @var button_clicked_2
        ## _ang_spe_button is the to the curosr position,
        ## when angular velocity command is requesting 
        self._ang_spe_button.setSizePolicy(size_policy)
        self._ang_spe_button._pixmap = QPixmap(package_directory +
                                               "/icons/led/green-on-48.png")
        self._ang_spe_button._icon = QIcon()
        self._ang_spe_button._icon.addPixmap(self._ang_spe_button._pixmap,
                                                     QIcon.Normal,
                                                     QIcon.Off)
        self._ang_spe_button.setIcon(self._ang_spe_button._icon)
        self._ang_spe_button.setIconSize(QSize(25, 25))
        self._ang_spe_button.setGeometry(QRect(0, 0, 25, 25))
        self._ang_spe_button.setFlat(True)
        self._ang_spe_button.setCheckable(False)
        self._ang_spe_button.hide()
        self._ang_spe_button.state = False
        
        self.setMouseTracking(True)
        
        rospy.loginfo("Omni_gui hydro version 4.0 started !!!")
        rospy.loginfo("Parameters : ")
        rospy.loginfo("MAX LINEAR SPEED : " + str(MAX_LIN_SPEED))
        rospy.loginfo("MAX ANGULAR SPEED : " + str(MAX_ANG_SPEED))
    
    ## @brief mousePressEvent redefinition
    ##
    ## @version 4.0
    ## @author Clement Beaudoing
    ## @date Last modification le 16/04/2014
    def mousePressEvent(self, event):
        self.setCursor(Qt.ClosedHandCursor)
        my_size = self._my_icon.actualSize(QSize(self.width(),
                                                 self.height()),
                                           QIcon.Normal,
                                           QIcon.Off)
        pix_width = my_size.width() #pixmap width (pix)
        pix_height = my_size.height() #pixmap height (pix)
        a = pix_width - pix_height # hauteur min libre autour du disque bleu
        R = (pix_width - 2*a) / 2 # Rayon du demi-disque bleu
        R_arc = pix_width / 2 # Rayon exterieur de l'anneau
        r_arc = R_arc - 78.0 * pix_width / 1000.0 # Rayon interieur de l'anneau
        x_origin = self.width()/2
        y_origin = a / 2 + self.height()/2
        x_current = event.pos().x() - x_origin 
        y_current = event.pos().y() - y_origin
        if (sqrt(x_current*x_current + y_current*y_current) < R) :
            pos_x_cursor = event.pos().x()-self._lin_spe_button.width()/2
            pos_y_cursor = event.pos().y()-self._lin_spe_button.height()/2
            self._lin_spe_button.move(pos_x_cursor, pos_y_cursor)
            self._lin_spe_button.state = True
            self._lin_spe_button.show()
#            x =  float(y_current) / float(my_size.width()) * 2.0
#            th = float(x_current) / float(my_size.width()) * 2.0
            x =  float(y_current) / R * MAX_LIN_SPEED
            th = float(x_current) / R * MAX_LIN_SPEED
            self._twist = Twist()
            self._twist.linear.x = -x
            self._twist.linear.y = -th
            self.emit(SIGNAL("twist"), self._twist)
            self.emit(SIGNAL('status'), ACTIVE)
        elif ((sqrt(x_current*x_current + y_current*y_current) < R_arc) and
              (x_current*x_current + y_current*y_current > r_arc*r_arc) and
              y_current < 0):
            pos_x_cursor = event.pos().x()-self._ang_spe_button.width()/2
            pos_y_cursor = event.pos().y()-self._ang_spe_button.height()/2
            self._ang_spe_button.move(pos_x_cursor, pos_y_cursor)
            self._ang_spe_button.state = True
            self._ang_spe_button.show()
#            th = float(x_current) / float(my_size.width()) * 2.0
            th = float(x_current) / R_arc * MAX_ANG_SPEED
            self._twist = Twist()
            self._twist.angular.z = -th
            self.emit(SIGNAL("twist"), self._twist)
            self.emit(SIGNAL('status'), ACTIVE)
            
    ## @brief mouseReleaseEvent redefinition
    ##
    ## @version 4.0
    ## @author Clement Beaudoing
    ## @date Last modification le 16/04/2014
    def mouseReleaseEvent(self, event):
        self.setCursor(Qt.OpenHandCursor)
        self._lin_spe_button.hide()
        self._lin_spe_button.state = False
        self._ang_spe_button.hide()
        self._ang_spe_button.state = False
        self._twist = Twist()
        self.emit(SIGNAL("twist"), self._twist)
        self.emit(SIGNAL('status'), PAUSE)
        
    ## @brief mouseMoveEvent redefinition
    ##
    ## @version 4.0
    ## @author Clement Beaudoing
    ## @date Last modification le 16/04/2014
    def mouseMoveEvent(self, event):
        my_size = self._my_icon.actualSize(QSize(self.width(),
                                                 self.height()),
                                           QIcon.Normal,
                                           QIcon.Off)
        pix_width = my_size.width()  #pixmap width (pix)
        pix_height = my_size.height() #pixmap height (pix)
        a = pix_width - pix_height # hauteur min libre autour du disque bleu
        R = (pix_width - 2*a) / 2 # Rayon du demi-disque bleu
        R_arc = pix_width / 2 # Rayon exterieur de l'anneau
        r_arc = R_arc - 78.0 * pix_width / 1000.0 # Rayon interieur de l'anneau
        x_origin = self.width()/2
        y_origin = a / 2 + self.height()/2
        x_current = event.pos().x() - x_origin 
        y_current = event.pos().y() - y_origin
        if (sqrt(x_current*x_current + y_current*y_current) < R) :
            self.setCursor(Qt.OpenHandCursor)
            if self._lin_spe_button.state == True:
                self.setCursor(Qt.ClosedHandCursor)
                pos_x_cursor = event.pos().x()-self._lin_spe_button.width()/2
                pos_y_cursor = event.pos().y()-self._lin_spe_button.height()/2
                self._lin_spe_button.move(pos_x_cursor, pos_y_cursor)
#                x =  float(y_current) / float(my_size.width()) * 2.0
#                th = float(x_current) / float(my_size.width()) * 2.0
                x =  float(y_current) / R * MAX_LIN_SPEED
                th = float(x_current) / R * MAX_LIN_SPEED
                self._twist = Twist()
                self._twist.linear.x = -x
                self._twist.linear.y = -th
                self.emit(SIGNAL("twist"), self._twist)
                self.emit(SIGNAL('status'), ACTIVE)
        elif ((sqrt(x_current*x_current + y_current*y_current) < R_arc) and
              (sqrt(x_current*x_current + y_current*y_current) > r_arc) and
              y_current < 0):
            self.setCursor(Qt.OpenHandCursor)
            if self._ang_spe_button.state == True:
                self.setCursor(Qt.ClosedHandCursor)
                pos_x_cursor = event.pos().x()-self._ang_spe_button.width()/2
                pos_y_cursor = event.pos().y()-self._ang_spe_button.height()/2
                self._ang_spe_button.move(pos_x_cursor, pos_y_cursor)
#                th = float(x_current) / float(my_size.width()) * 2.0
                th = float(x_current) / R_arc * MAX_ANG_SPEED
                self._twist = Twist()
                self._twist.angular.z = -th
                self.emit(SIGNAL("twist"), self._twist)
                self.emit(SIGNAL('status'), ACTIVE)
                
    ## @brief resizeEvent redefinition
    ##
    ## @version 4.0
    ## @author Clement Beaudoing
    ## @date Last modification le 16/04/2014
    def resizeEvent(self, event):
        self.setIconSize(QSize(self.width(), self.height()))

## @class Ui_OmniGUI
##
## @author Clement Beaudoing
## @date Last modification le 16/04/2014
class Ui_OmniGUI(QWidget):
    def __init__(self, parent=None):
        super(Ui_OmniGUI, self).__init__(parent)
        ## @var pub 
        self._pub = rospy.Publisher('cmd_vel', Twist, latch = True, queue_size=1)
        ## @var _twist: 
        self._twist = Twist()
        ## @var _central_screen
        self._central_screen = CustomCentralScreen()
        ## @var _cmd_publisher: 
        self._cmd_publisher = ThreadPublisher()
        
    ## @brief Ui_OmniGUI initialization
    ## 
    ## @version 4.0
    ## @author Clement Beaudoing
    ## @date Last modification le 16/04/2014
    def setup_ui(self):
        package_directory = roslib.packages.get_pkg_dir('omni_gui')
        
        #self.setObjectName("OmniGUI")
        self.resize(803, 595)
        title = "AIRBUS GROUP Innovations Graphical User Interface v4.0 "
        self.setWindowTitle(QApplication.translate("OmniGUI",
                                                   title,
                                                   None,
                                                   QApplication.UnicodeUTF8))
        #self.main_window = QMainWindow() 
        ## @var main_window 
        icon = QIcon()
        icon.addPixmap(QPixmap(package_directory +
                               "/icons/AIRBUS_Group_short2.png"),
                       QIcon.Normal,
                       QIcon.Off)
        self.setWindowIcon(icon)
        
        #Organisations visuel des elements.
        horizontal_layout = QVBoxLayout(self)
        grid_layout = QGridLayout()
        grid_layout.addWidget(self._central_screen, 0, 0, 1, 1)
        grid_layout.setSizeConstraint(QLayout.SetNoConstraint)
        horizontal_layout.addLayout(grid_layout)
                
        self._cmd_publisher.start()
        
        self.connect(self._cmd_publisher,
                     SIGNAL('cmd_vel'),
                     self.pub_twist)
        self.connect(self._central_screen,
                     SIGNAL("twist"),
                     self.twist_saver)
        self.connect(self._central_screen,
                     SIGNAL('status'),
                     self._cmd_publisher.thread_status)
        
    
    ## @brief twist value recorder
    ## 
    ## @version 4.0
    ## @author Clement Beaudoing
    ## @date Last modification le 16/04/2014
    def twist_saver(self, twist):
        self._twist = twist
        
    ## @brief /cmd_vel topic publisher
    ## 
    ## @version 4.0
    ## @author Clement Beaudoing
    ## @date Last modification le 16/04/2014
    def pub_twist(self):
        self._pub.publish(self._twist)
        
    def shutdown(self):
        #Stop publisher thread
        self._cmd_publisher.stop()
        #Unregister cmd_vel publisher
        self._pub.unregister()
        
        time.sleep(0.2)

