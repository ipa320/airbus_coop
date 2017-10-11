#!/usr/bin/env python
#
# Copyright 2015 Airbus
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
import sys
from roslib.packages import get_pkg_dir

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi

from accounts import User, \
                     Privilege, \
                     UserAccounts, \
                     base64
                     
from airbus_pyqt_extend.QtAgiGui import QAgiPopup, QAgiMessageBox

from airbus_cobot_gui.dashboard import Dashboard, DashboardPopup

from airbus_cobot_gui.res import R

## @package: user_account
##
## @version 2.2
## @author  Matignon Martin
## @date    Last modified 22/05/2014

## @class LoginDialog
## @brief Login user interface.
class LoginDialog(QDialog):
    
    def __init__(self, parent, closable = True):
        """! The constructor."""
        QDialog.__init__(self, parent, Qt.FramelessWindowHint)
        
        self._context = parent.getContext()
        self._lng = self._context.getLanguage()
        
        # Extend the widget with all attributes and children from UI file
        loadUi(R.layouts.login_dialog, self)
        
        self.setStyleSheet(R.values.styles.login)
        
        self.setModal(True)
        self.adjustSize()
        
        self.login_button.setText(R.values.strings.login(self._lng))
        self.exit_button.setText(R.values.strings.exit(self._lng))
        
        self.user_id_label.setText(R.values.strings.user_id(self._lng))
        self.password_label.setText(R.values.strings.password(self._lng))
        
        self.password_edit.setEchoMode(QLineEdit.Password)
        
        self.connect(self.login_button, SIGNAL("clicked()"),
                      self.account_validation)
        
        if closable:
            self.connect(self.exit_button, SIGNAL("clicked()"), self.close)
            self.exit_button.setVisible(True)
        else:
            self.exit_button.setVisible(False)
        
        self.adjustSize()
    
    def account_validation(self):
        """! Check user account validity and connect user."""
        
        accounts = UserAccounts()
        
        user = accounts.find(self.user_id_edit.text())
        
        if user is None:
            msg_box = QAgiMessageBox()
            msg_box.setText(R.values.strings.invalid_user_id(self._lng))
            msg_box.setIcon(QAgiMessageBox.Critical)
            msg_box.setStandardButtons(QAgiMessageBox.Ok)
            msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
            msg_box.exec_()
        elif user.password == base64.b64encode(self.password_edit.text()):
            self._context.switchUser(user)
            self.close()
        else:
            msg_box = QAgiMessageBox()
            msg_box.setText(R.values.strings.invalid_password(self._lng))
            msg_box.setIcon(QAgiMessageBox.Critical)
            msg_box.setStandardButtons(QAgiMessageBox.Ok)
            msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
            msg_box.exec_()
        
## @class AddUserAccountWidget
## @brief Add user account ui.
class AddUserAccountWidget(QWidget):
    
    def __init__(self, parent):
        """! The constructor."""
        QWidget.__init__(self)
        
        self._context = parent.getContext()
        self._lng = self._context.getLanguage()
        
        # Extend the widget with all attributes and children from UI file
        loadUi(R.layouts.add_user_widget, self)
        
        self.user_id_label.setText(R.values.strings.user_id(self._lng))
        self.password_label.setText(R.values.strings.password(self._lng))
        self.cinfirm_password_label.setText(R.values.strings.confirm_password(self._lng))
        self.access_rights_label.setText(R.values.strings.access_rights(self._lng))
        self.add_user_button.setText(R.values.strings.ok(self._lng))
        
        self.password_edit.setEchoMode(QLineEdit.Password)
        self.confirm_password_edit.setEchoMode(QLineEdit.Password);
        
        self.connect(self.add_user_button, SIGNAL("clicked()"),
                      self.add_user_account)
        
    def add_user_account(self):
        """! Check fields and add user account."""
        
        accounts = UserAccounts()
        
        user = User(self.user_id_edit.text())
        
        password = self.password_edit.text()
        confirm = self.confirm_password_edit.text()
        
        if user.userid == "" or \
           password == "" or \
           confirm == "":
            msg_box = QAgiMessageBox()
            msg_box.setText(R.values.strings.fields_not_filled(self._lng))
            msg_box.setIcon(QAgiMessageBox.Critical)
            msg_box.setStandardButtons(QAgiMessageBox.Ok)
            msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
            msg_box.exec_()
            return
        
        if confirm != password:
            msg_box = QAgiMessageBox()
            msg_box.setText(R.values.strings.passwords_different(self._lng))
            msg_box.setIcon(QAgiMessageBox.Critical)
            msg_box.setStandardButtons(QAgiMessageBox.Ok)
            msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
            msg_box.exec_()
            return
        else:
            user.setUserPassword(password)
        
        user.setUserPrivilege(Privilege.TOLEVEL[self.access_rights_combo.currentText().lower()])
        
        try:
            accounts.add(user)
        except Exception as e:
            msg_box = QAgiMessageBox()
            msg_box.setText(str(e))
            msg_box.setIcon(QAgiMessageBox.Critical)
            msg_box.setStandardButtons(QAgiMessageBox.Ok)
            msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
            msg_box.exec_()
            return
        
        msg_box = QAgiMessageBox()
        msg_box.setText(R.values.strings.add_user_success(self._lng))
        msg_box.setIcon(QAgiMessageBox.Information)
        msg_box.setStandardButtons(QAgiMessageBox.Ok)
        msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
        msg_box.exec_()
        
        self.user_id_edit.setText("")
        self.password_edit.setText("")
        self.confirm_password_edit.setText("")
        self.access_rights_combo.setCurrentIndex(0)
        

## @class ModifUserAccountWidget
## @brief Modif user account ui.
class ModifUserAccountWidget(QWidget):
    
    def __init__(self, parent):
        """! The constructor."""
        QWidget.__init__(self)
        
        self._context = parent.getContext()
        self._lng = self._context.getLanguage()
        
        # Extend the widget with all attributes and children from UI file
        loadUi(R.layouts.modif_account_widget, self)
        
        self.select_user_label.setText(R.values.strings.select_user(self._lng))
        self.current_password_label.setText(R.values.strings.current_password(self._lng))
        self.check_password_button.setText(R.values.strings.ok(self._lng))
        self.access_rights_label.setText(R.values.strings.access_rights(self._lng))
        self.new_password_label.setText(R.values.strings.new_password(self._lng))
        self.confirm_password_label.setText(R.values.strings.confirm_password(self._lng))
        self.modif_user_account_button.setText(R.values.strings.ok(self._lng))
        
        self.new_password_edit.setEchoMode(QLineEdit.Password)
        self.confirm_new_password_edit.setEchoMode(QLineEdit.Password)
        
        self._accounts = UserAccounts()
        self.user_selected = User()
        self.user_dst = User()
        
        self.users_list_combo.addItems(self._accounts.user_list())
        self.users_list_combo.currentIndexChanged.connect(self.update_user_info)
        
        self.connect(self.modif_user_account_button, SIGNAL("clicked()"),
                      self.modif_user_account)
        
        self.connect(self.check_password_button, SIGNAL("clicked()"),
                     self.check_password)
        
        self.current_password_edit.textChanged[str].connect(self.current_password_changed)
        
    def current_password_changed(self):
        self.init_fields(False)
        
    def check_password(self):
        
        password = self.current_password_edit.text()
        
        if password == self.user_selected.getUserPassword(True):
            
            self.current_password_edit.setStyleSheet(R.values.styles.good_password)
            
            privilege = self.user_selected.getUserPrivilege()
            self.access_rights_combo.setCurrentIndex(privilege+1)
            self.new_password_edit.setText(self.user_selected.getUserPassword(True))
            self.confirm_new_password_edit.setText(self.user_selected.getUserPassword(True))
            
            self.new_password_edit.setEnabled(True)
            self.confirm_new_password_edit.setEnabled(True)
            self.access_rights_combo.setEnabled(True)
            self.modif_user_account_button.setEnabled(True)
            
        else:
            
            self.init_fields(False)
            
            self.current_password_edit.setStyleSheet(R.values.styles.bad_password)
            
        
    def update_user_info(self, index):
        """! Update user information with user selected.
        @param index: user list index.
        @type index: int.
        """
        self.init_fields()
        
        if index == 0:
            self.current_password_edit.setEnabled(False)
        else:
            self.user_selected = self._accounts.find(self.users_list_combo.itemText(index))
            self.current_password_edit.setEnabled(True)
            
    def modif_user_account(self):
        """! Check fields and modify user account."""
        
        user_modified = User(self.user_selected.getUserId())
        
        if self.access_rights_combo.currentIndex() == 0:
            
            msg_box = QAgiMessageBox()
            msg_box.setText(R.values.strings.select_access_rights(self._lng))
            msg_box.setIcon(QAgiMessageBox.Critical)
            msg_box.setStandardButtons(QAgiMessageBox.Ok)
            msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
            msg_box.exec_()
            return
        elif self.new_password_edit.text() != \
             self.confirm_new_password_edit.text():
            
            msg_box = QAgiMessageBox()
            msg_box.setText(R.values.strings.passwords_different(self._lng))
            msg_box.setIcon(QAgiMessageBox.Critical)
            msg_box.setStandardButtons(QAgiMessageBox.Ok)
            msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
            msg_box.exec_()
            return
        else:
            user_modified.privilege = Privilege.TOLEVEL[self.access_rights_combo.currentText().lower()]
            user_modified.setUserPassword(self.new_password_edit.text())
        
        try:
            
            self._accounts.modif(self.user_selected, user_modified)
        except Exception as e:
            
            msg_box = QAgiMessageBox()
            msg_box.setText(str(e))
            msg_box.setIcon(QAgiMessageBox.Critical)
            msg_box.setStandardButtons(QAgiMessageBox.Ok)
            msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
            msg_box.exec_()
            return
            
        msg_box = QAgiMessageBox()
        msg_box.setText(R.values.strings.user_mv_success(self._lng))
        msg_box.setIcon(QAgiMessageBox.Information)
        msg_box.setStandardButtons(QAgiMessageBox.Ok)
        msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
        msg_box.exec_()
        
        self.users_list_combo.setCurrentIndex(0)
        self.init_fields()
        
    def init_fields(self, clear_all = True):
        
        self.access_rights_combo.setCurrentIndex(0)
        self.new_password_edit.setText('')
        self.confirm_new_password_edit.setText('')
        self.new_password_edit.setEnabled(False)
        self.confirm_new_password_edit.setEnabled(False)
        self.access_rights_combo.setEnabled(False)
        self.modif_user_account_button.setEnabled(False)
        
        if clear_all:
            self.current_password_edit.setText('')
            
        self.current_password_edit.setStyleSheet(R.values.styles.no_password)
        
## @class RemoveUserAccountWidget
## @brief Remove user account ui.
class RemoveUserAccountWidget(QWidget):
    
    def __init__(self, parent):
        """! The constructor."""
        
        QWidget.__init__(self)
        # Extend the widget with all attributes and children from UI file
        loadUi(R.layouts.remove_account_widget, self)
        
        self._context = parent.getContext()
        self._lng = self._context.getLanguage()
        
        self.user_list_label.setText(R.values.strings.user_list(self._lng))
        self.remove_button.setText(R.values.strings.ok(self._lng))
        
        self.connect(self.remove_button, SIGNAL("clicked()"),
                      self.remove_account)
         
        self._accounts = UserAccounts()
         
        self.users_list_combo.addItems(self._accounts.user_list())
        
    def remove_account(self):
        """! Remove user account slected in user list."""
        
        if self.users_list_combo.currentIndex() == 0:
            msg_box = QAgiMessageBox()
            msg_box.setText(R.values.strings.select_user(self._lng))
            msg_box.setIcon(QAgiMessageBox.Critical)
            msg_box.setStandardButtons(QAgiMessageBox.Ok)
            msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
            msg_box.exec_()
            return
        
        user_id = self.users_list_combo.currentText()
        try:
            self._accounts.remove(User(user_id))
            self.users_list_combo.removeItem(self.users_list_combo.currentIndex())
        except Exception as e:
            msg_box = QAgiMessageBox()
            msg_box.setText(str(e))
            msg_box.setIcon(QAgiMessageBox.Critical)
            msg_box.setStandardButtons(QAgiMessageBox.Ok)
            msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
            msg_box.exec_()
            return
            
        self.users_list_combo.setCurrentIndex(0)
        msg_box = QAgiMessageBox()
        msg_box.setText(R.values.strings.user_rm_success(self._lng))
        msg_box.setIcon(QAgiMessageBox.Information)
        msg_box.setStandardButtons(QAgiMessageBox.Ok)
        msg_box.button(QAgiMessageBox.Ok).setMinimumSize(100,40)
        msg_box.exec_()
        
## @class UsersAccountsManagerDialog
## @brief User accounts manager ui.
class UsersAccountsManagerDialog(QDialog):
    
    def __init__(self, parent):
        """! The constructor."""
        QDialog.__init__(self, parent, Qt.FramelessWindowHint)
        
        self._parent = parent
        self._lng = self._parent.getContext().getLanguage()
        
        # Extend the widget with all attributes and children from UI file
        loadUi(R.layouts.users_accounts_dialog, self)
        
        self.setModal(True)
        
        self.settings_label.setText(R.values.strings.settings(self._lng))
        self.header_label.setText(R.values.strings.account_manager(self._lng))
        self.add_button.setText(R.values.strings.add(self._lng))
        self.modif_button.setText(R.values.strings.modif(self._lng))
        self.remove_button.setText(R.values.strings.remove(self._lng))
        self.exit_button.setText(R.values.strings.exit(self._lng))
        
        self.connect(self.exit_button, SIGNAL("clicked()"), self.close)
        self.connect(self.add_button, SIGNAL("clicked()"), self.load_add_user_ui)
        self.connect(self.modif_button, SIGNAL("clicked()"), self.load_modif_user_ui)
        self.connect(self.remove_button, SIGNAL("clicked()"), self.load_remove_user_ui)
        
        self.add_button.click()
        
    def load_add_user_ui(self):
        """! Open add user account ui."""
        self.header_label.setText(R.values.strings.add_user(self._lng))
        
        self.viewer_area.takeWidget()
        add_account_ui = AddUserAccountWidget(self._parent)
        add_account_ui.resize(self.viewer_area.width()-2,
                              self.viewer_area.height()-2)
        self.viewer_area.setWidget(add_account_ui)
    
    def load_modif_user_ui(self):
        """! Open modif user account ui."""
        self.header_label.setText(R.values.strings.modif_user_account(self._lng))
        
        self.viewer_area.takeWidget()
        modif_account_ui = ModifUserAccountWidget(self._parent)
        modif_account_ui.resize(self.viewer_area.width()-2,
                                self.viewer_area.height()-2)
        self.viewer_area.setWidget(modif_account_ui)
    
    def load_remove_user_ui(self):
        """! Open remove user account ui."""
        self.header_label.setText(R.values.strings.remove_user_account(self._lng))
        
        self.viewer_area.takeWidget()
        remove_account = RemoveUserAccountWidget(self._parent)
        remove_account.resize(self.viewer_area.width()-2,
                              self.viewer_area.height()-2)
        self.viewer_area.setWidget(remove_account)
        
## @class UserAccountPopup
## @brief User accounts popup ui.
class UserAccountPopup(DashboardPopup):
    
    def __init__(self, parent):
        """! The constructor."""
        DashboardPopup.__init__(self, parent)
        
        self.setRelativePosition(DashboardPopup.TopRight,
                                 DashboardPopup.BottomRight)
        
    def onCreate(self, param):
        # Extend the widget with all attributes and children from UI file
        loadUi(R.layouts.account_popup, self)
        
        self.user_icon_label.setPixmap(R.getPixmapById('ico_user'))
        
        user_info = self.getParent().getContext().getUserInfo()
        
        if user_info.privilege < Privilege.EXPERT:
            self.accounts_manager_button.setEnabled(False)
        else:
            self.accounts_manager_button.setEnabled(True)
        
        self.user_id_label.setText(user_info.userid)
        
        privilege = Privilege.TOSTR[user_info.privilege]
        self.access_rights_label.setText(privilege[0].upper()+privilege[1:])
        self.time_label.setText('2h:38m')
        
        self.connect(self.connection_button, SIGNAL("clicked()"),
                     self.open_login_dialog)
        self.connect(self.accounts_manager_button, SIGNAL("clicked()"), 
                     self.open_accounts_manager_dialog)
        self.connect(self.deconnection_button, SIGNAL("clicked()"), 
                     self.disconnect_user_account)
        self.deconnection_button.setEnabled(True)
        
        self.adjustSize()
        
    def _resfresh_connection_time(self):
        """! Refresh connection time."""
        self.time_label.setText(self._user.connection_time())
        
    def open_login_dialog(self):
        """! Open login ui."""
        login = LoginDialog(self.getParent())
        login.show()
        
    def open_accounts_manager_dialog(self):
        """! Open account manager ui."""
        manager = UsersAccountsManagerDialog(self.getParent())
        manager.show()
        
    def disconnect_user_account(self):
        """! Disconnect current user."""
        login = LoginDialog(self.getParent(), closable = False)
        login.show()
        
    def onTranslate(self, lng):
        
        self.header_label.setText(R.values.strings.user_account(lng))
        self.user_id_header_label.setText(R.values.strings.user(lng))
        self.access_rights_header_label.setText(R.values.strings.rights(lng))
        self.language_header_label.setText(R.values.strings.language(lng))
        self.time_header_label.setText(R.values.strings.time(lng))
        self.connection_button.setText(R.values.strings.connection(lng))
        self.deconnection_button.setText(R.values.strings.disconnection(lng))
        self.accounts_manager_button.setText(R.values.strings.account_manager(lng))
        self.language_label.setText(R.values.strings.language(lng))
        
    def onDestroy(self):
        pass

## @class UserAccountsWidget
## @brief User connected information displaying on dashboard.
class UserAccountsWidget(Dashboard):
     
    def __init__(self, context):
        Dashboard.__init__(self, context)
    
    def onCreate(self, param):
        
        self._user_icon_label = QLabel()
        self._user_icon_label.setStyleSheet(R.values.styles.text)
        self._user_icon_label.setFixedSize(QSize(25,25))
        self._user_icon_label.setPixmap(R.getPixmapById('ico_user').scaled(
                                   self._user_icon_label.width(),
                                   self._user_icon_label.height(),
                                   Qt.KeepAspectRatio,
                                   Qt.SmoothTransformation))
        ######
         
        self._user_id_label = QLabel()
        self._user_id_label.setStyleSheet(R.values.styles.transparent_background)
         
        ######
         
        self.getLayout().addWidget(self._user_icon_label)
        self.getLayout().addWidget(self._user_id_label)
         
    def onUserChanged(self, user):
        """! Update user information.
        @param user_info: user connected information.
        @type user_info: UserInfo.
        """
        self._user_id_label.setText(user.userid)
        
    def onControlModeChanged(self, mode):
        pass
    
    def onRequestPopup(self):
        return UserAccountPopup(self)
         
    def onTranslate(self):
        pass
    
    def onEmergencyStop(self, state):
        pass
    
    def onDestroy(self):
        pass
        
    
#End of file
