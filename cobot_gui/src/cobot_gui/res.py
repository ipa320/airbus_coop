#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : res.py
# Authors : Martin Matignon
#
################################################################################


########################################
# Module(s) declaration
########################################

import rospy
import os
from roslib.packages import get_pkg_dir
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding import loadUi

########################################
# Constante(s) and Variable(s) declaration
########################################

DIR_COBOTGUI_RESOURCES = os.path.join(get_pkg_dir('cobot_gui'),'resources')
DIR_COBOTGUI_ACCOUNTS = DIR_COBOTGUI_RESOURCES+'/accounts'
DIR_COBOTGUI_BACKUP = DIR_COBOTGUI_RESOURCES+'/accounts/backup'
DIR_COBOTGUI_IMAGES = DIR_COBOTGUI_RESOURCES+'/images'
DIR_COBOTGUI_LAYOUTS = DIR_COBOTGUI_RESOURCES+'/layouts'
DIR_COBOTGUI_VALUES = DIR_COBOTGUI_RESOURCES+'/values'

########################################
# Class(ies) declaration
########################################

class CobotGuiAccounts():
    def __init__(self):
        class CobotGuiBackup():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.dir = DIR_COBOTGUI_BACKUP
                self.accounts_back = DIR_COBOTGUI_BACKUP+'/accounts_back.db'
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_COBOTGUI_ACCOUNTS
        self.accounts = DIR_COBOTGUI_ACCOUNTS+'/accounts.db'
        self.accounts = DIR_COBOTGUI_ACCOUNTS+'/accounts.xml'
        self.encoded_accounts = DIR_COBOTGUI_ACCOUNTS+'/encoded_accounts.db'
        self.backup = CobotGuiBackup()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class CobotGuiImages():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_COBOTGUI_IMAGES
        self.icon_pause = DIR_COBOTGUI_IMAGES+'/icon_pause.png'
        self.wellcome_background = DIR_COBOTGUI_IMAGES+'/wellcome_background.png'
        self.trad = DIR_COBOTGUI_IMAGES+'/trad.png'
        self.logo_airbus_group_2 = DIR_COBOTGUI_IMAGES+'/logo_airbus_group_2.png'
        self.open_xml = DIR_COBOTGUI_IMAGES+'/open_xml.png'
        self.ico_user = DIR_COBOTGUI_IMAGES+'/ico_user.png'
        self.es = DIR_COBOTGUI_IMAGES+'/es.png'
        self.en = DIR_COBOTGUI_IMAGES+'/en.png'
        self.fr = DIR_COBOTGUI_IMAGES+'/fr.png'
        self.de = DIR_COBOTGUI_IMAGES+'/de.png'
        self.logo_airbus = DIR_COBOTGUI_IMAGES+'/logo_airbus.png'
        self.icon_play = DIR_COBOTGUI_IMAGES+'/icon_play.png'
        self.icon_mission = DIR_COBOTGUI_IMAGES+'/icon_mission.png'
        self.move = DIR_COBOTGUI_IMAGES+'/move.png'
        self.icon_monitoring = DIR_COBOTGUI_IMAGES+'/icon_monitoring.png'
        self.desktop_launch = DIR_COBOTGUI_IMAGES+'/desktop_launch.png'
        self.cobot_gui = DIR_COBOTGUI_IMAGES+'/cobot_gui.png'
        self.switch_footer = DIR_COBOTGUI_IMAGES+'/switch_footer.png'
        self.ico_alarm = DIR_COBOTGUI_IMAGES+'/ico_alarm.png'
        self.icon_emergency_stop_unlock = DIR_COBOTGUI_IMAGES+'/icon_emergency_stop_unlock.png'
        self.icon_emergency_stop_locked = DIR_COBOTGUI_IMAGES+'/icon_emergency_stop_locked.png'
        self.logo_airbus_group = DIR_COBOTGUI_IMAGES+'/logo_airbus_group.png'
        self.wellcome_base = DIR_COBOTGUI_IMAGES+'/wellcome_base.png'
        self.icon_emergency_stop = DIR_COBOTGUI_IMAGES+'/icon_emergency_stop.png'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class CobotGuiLayouts():
    def __init__(self):
        self.uuid = self.__class__.__name__
        self.dir = DIR_COBOTGUI_LAYOUTS
        self.languages_manager_popup = DIR_COBOTGUI_LAYOUTS+'/languages_manager_popup.ui'
        self.add_user_widget = DIR_COBOTGUI_LAYOUTS+'/add_user_widget.ui'
        self.account_popup = DIR_COBOTGUI_LAYOUTS+'/account_popup.ui'
        self.menu_launcher2 = DIR_COBOTGUI_LAYOUTS+'/menu_launcher2.ui'
        self.remove_account_widget = DIR_COBOTGUI_LAYOUTS+'/remove_account_widget.ui'
        self.login_dialog = DIR_COBOTGUI_LAYOUTS+'/login_dialog.ui'
        self.alarm_widget = DIR_COBOTGUI_LAYOUTS+'/alarm_widget.ui'
        self.mainwindow = DIR_COBOTGUI_LAYOUTS+'/mainwindow.ui'
        self.users_accounts_dialog = DIR_COBOTGUI_LAYOUTS+'/users_accounts_dialog.ui'
        self.welcome = DIR_COBOTGUI_LAYOUTS+'/welcome.ui'
        self.accounts_manager_dialog = DIR_COBOTGUI_LAYOUTS+'/accounts_manager_dialog.ui'
        self.modif_account_widget = DIR_COBOTGUI_LAYOUTS+'/modif_account_widget.ui'
        self.alarm_listview = DIR_COBOTGUI_LAYOUTS+'/alarm_listview.ui'
        self.languages_popup = DIR_COBOTGUI_LAYOUTS+'/languages_popup.ui'
        self.menu_launcher = DIR_COBOTGUI_LAYOUTS+'/menu_launcher.ui'
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class CobotGuiValues():
    def __init__(self):
        class CobotGuiStrings():
            def __init__(self):
                self.uuid = self.__class__.__name__
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
            def alarms_waiting(self, lng="en"):
                if lng == "en":
                    return "alarms waiting ...".decode('utf-8')
                elif lng == "fr":
                    return "alarmes en attente ...".decode('utf-8')
                else:
                    return "alarms waiting ...".decode('utf-8')
                
            def language_selection(self, lng="en"):
                if lng == "en":
                    return "The language selected is".decode('utf-8')
                elif lng == "fr":
                    return "La langue sélectionnée est".decode('utf-8')
                elif lng == "de":
                    return "Die gewählte Sprache".decode('utf-8')
                elif lng == "es":
                    return "El idioma seleccionado es".decode('utf-8')
                else:
                    return "The language selected is".decode('utf-8')
                
            def app_mode(self, lng="en"):
                if lng == "en":
                    return "Switching AUTOMATIC to MANUAL mode".decode('utf-8')
                elif lng == "fr":
                    return "Commutation du mode automatique à manuel".decode('utf-8')
                else:
                    return "Switching AUTOMATIC to MANUAL mode".decode('utf-8')
                
            def auto(self, lng="en"):
                if lng == "en":
                    return "AUTO".decode('utf-8')
                elif lng == "fr":
                    return "AUTO".decode('utf-8')
                else:
                    return "AUTO".decode('utf-8')
                
            def manu(self, lng="en"):
                if lng == "en":
                    return "MANU".decode('utf-8')
                elif lng == "fr":
                    return "MANU".decode('utf-8')
                else:
                    return "MANU".decode('utf-8')
                
            def aborted(self, lng="en"):
                if lng == "en":
                    return "Aborted".decode('utf-8')
                elif lng == "fr":
                    return "Avorté".decode('utf-8')
                else:
                    return "Aborted".decode('utf-8')
                
            def access_rights(self, lng="en"):
                if lng == "en":
                    return "Access rights".decode('utf-8')
                elif lng == "fr":
                    return "Droits d'accès".decode('utf-8')
                else:
                    return "Access rights".decode('utf-8')
                
            def account_manager(self, lng="en"):
                if lng == "en":
                    return "Account manager".decode('utf-8')
                elif lng == "fr":
                    return "Gestion des comptes".decode('utf-8')
                else:
                    return "Account manager".decode('utf-8')
                
            def actions(self, lng="en"):
                if lng == "en":
                    return "Actions".decode('utf-8')
                elif lng == "fr":
                    return "Actions".decode('utf-8')
                else:
                    return "Actions".decode('utf-8')
                
            def add(self, lng="en"):
                if lng == "en":
                    return "Add".decode('utf-8')
                elif lng == "fr":
                    return "Ajouter".decode('utf-8')
                else:
                    return "Add".decode('utf-8')
                
            def add_user(self, lng="en"):
                if lng == "en":
                    return "Add user account".decode('utf-8')
                elif lng == "fr":
                    return "Ajouter compte utilisateur".decode('utf-8')
                else:
                    return "Add user account".decode('utf-8')
                
            def confirm_password(self, lng="en"):
                if lng == "en":
                    return "Confirm password".decode('utf-8')
                elif lng == "fr":
                    return "Confirmer le mot de passe".decode('utf-8')
                else:
                    return "Confirm password".decode('utf-8')
                
            def confirm_your_password(self, lng="en"):
                if lng == "en":
                    return "Confirm your password".decode('utf-8')
                elif lng == "fr":
                    return "Confirmez votre mot de passe".decode('utf-8')
                else:
                    return "Confirm your password".decode('utf-8')
                
            def connection(self, lng="en"):
                if lng == "en":
                    return "Connection".decode('utf-8')
                elif lng == "fr":
                    return "Connection".decode('utf-8')
                else:
                    return "Connection".decode('utf-8')
                
            def current(self, lng="en"):
                if lng == "en":
                    return "Current".decode('utf-8')
                elif lng == "fr":
                    return "Courant".decode('utf-8')
                else:
                    return "Current".decode('utf-8')
                
            def current_password(self, lng="en"):
                if lng == "en":
                    return "Current password".decode('utf-8')
                elif lng == "fr":
                    return "Mot de passe actuel".decode('utf-8')
                else:
                    return "Current password".decode('utf-8')
                
            def emergency_stop(self, lng="en"):
                if lng == "en":
                    return "The system was stopped, be careful before restarting the applications !".decode('utf-8')
                elif lng == "fr":
                    return "Le système a été arrêt, faite attention avant de redémarrer les applications !".decode('utf-8')
                else:
                    return "The system was stopped, be careful before restarting the applications !".decode('utf-8')
                
            def release_emergency_stop(self, lng="en"):
                if lng == "en":
                    return "Release emergency stop".decode('utf-8')
                elif lng == "fr":
                    return "Déverrouillé l'arrêt d'urgence".decode('utf-8')
                else:
                    return "Release emergency stop".decode('utf-8')
                
            def disconnection(self, lng="en"):
                if lng == "en":
                    return "Disconnection".decode('utf-8')
                elif lng == "fr":
                    return "Déconnection".decode('utf-8')
                else:
                    return "Disconnection".decode('utf-8')
                
            def exit(self, lng="en"):
                if lng == "en":
                    return "Exit".decode('utf-8')
                elif lng == "fr":
                    return "Quitter".decode('utf-8')
                else:
                    return "Exit".decode('utf-8')
                
            def invalid_password(self, lng="en"):
                if lng == "en":
                    return "Invalid password !".decode('utf-8')
                elif lng == "fr":
                    return "Mot de passe incorrect !".decode('utf-8')
                else:
                    return "Invalid password !".decode('utf-8')
                
            def invalid_user_id(self, lng="en"):
                if lng == "en":
                    return "Invalid user id !".decode('utf-8')
                elif lng == "fr":
                    return "Identifiant utilisateur incorrect !".decode('utf-8')
                else:
                    return "Invalid user id !".decode('utf-8')
                
            def launch(self, lng="en"):
                if lng == "en":
                    return "Launch".decode('utf-8')
                elif lng == "fr":
                    return "Lanceur".decode('utf-8')
                else:
                    return "Launch".decode('utf-8')
                
            def login(self, lng="en"):
                if lng == "en":
                    return "Login".decode('utf-8')
                elif lng == "fr":
                    return "Connexion".decode('utf-8')
                else:
                    return "Login".decode('utf-8')
                
            def modif(self, lng="en"):
                if lng == "en":
                    return "Modif".decode('utf-8')
                elif lng == "fr":
                    return "Modifier".decode('utf-8')
                else:
                    return "Modif".decode('utf-8')
                
            def modif_user_account(self, lng="en"):
                if lng == "en":
                    return "Modif user account".decode('utf-8')
                elif lng == "fr":
                    return "Modifier compte utilisateur".decode('utf-8')
                else:
                    return "Modif user account".decode('utf-8')
                
            def name(self, lng="en"):
                if lng == "en":
                    return "Name".decode('utf-8')
                elif lng == "fr":
                    return "Nom".decode('utf-8')
                else:
                    return "Name".decode('utf-8')
                
            def new_password(self, lng="en"):
                if lng == "en":
                    return "New password".decode('utf-8')
                elif lng == "fr":
                    return "Nouveau mot de passe".decode('utf-8')
                else:
                    return "New password".decode('utf-8')
                
            def next_step(self, lng="en"):
                if lng == "en":
                    return "Next step".decode('utf-8')
                elif lng == "fr":
                    return "Etape suivante".decode('utf-8')
                else:
                    return "Next step".decode('utf-8')
                
            def off(self, lng="en"):
                if lng == "en":
                    return "Off".decode('utf-8')
                elif lng == "fr":
                    return "Off".decode('utf-8')
                else:
                    return "Off".decode('utf-8')
                
            def ok(self, lng="en"):
                if lng == "en":
                    return "Ok".decode('utf-8')
                elif lng == "fr":
                    return "Ok".decode('utf-8')
                else:
                    return "Ok".decode('utf-8')
                
            def on(self, lng="en"):
                if lng == "en":
                    return "On".decode('utf-8')
                elif lng == "fr":
                    return "On".decode('utf-8')
                else:
                    return "On".decode('utf-8')
                
            def open(self, lng="en"):
                if lng == "en":
                    return "Open".decode('utf-8')
                elif lng == "fr":
                    return "Ouvrir".decode('utf-8')
                else:
                    return "Open".decode('utf-8')
                
            def close(self, lng="en"):
                if lng == "en":
                    return "Close".decode('utf-8')
                elif lng == "fr":
                    return "Fermer".decode('utf-8')
                else:
                    return "Close".decode('utf-8')
                
            def open_mission(self, lng="en"):
                if lng == "en":
                    return "Open mission".decode('utf-8')
                elif lng == "fr":
                    return "Ouvrir une mission".decode('utf-8')
                else:
                    return "Open mission".decode('utf-8')
                
            def parameters(self, lng="en"):
                if lng == "en":
                    return "Parameters".decode('utf-8')
                elif lng == "fr":
                    return "Paramètres".decode('utf-8')
                else:
                    return "Parameters".decode('utf-8')
                
            def password(self, lng="en"):
                if lng == "en":
                    return "Password".decode('utf-8')
                elif lng == "fr":
                    return "Mot de passe".decode('utf-8')
                else:
                    return "Password".decode('utf-8')
                
            def preempt(self, lng="en"):
                if lng == "en":
                    return "Preempt".decode('utf-8')
                elif lng == "fr":
                    return "Préempter".decode('utf-8')
                else:
                    return "Preempt".decode('utf-8')
                
            def remove(self, lng="en"):
                if lng == "en":
                    return "Remove".decode('utf-8')
                elif lng == "fr":
                    return "Supprimer".decode('utf-8')
                else:
                    return "Remove".decode('utf-8')
                
            def remove_user_account(self, lng="en"):
                if lng == "en":
                    return "Remove user account".decode('utf-8')
                elif lng == "fr":
                    return "Supprimer compte utilisateur".decode('utf-8')
                else:
                    return "Remove user account".decode('utf-8')
                
            def rights(self, lng="en"):
                if lng == "en":
                    return "Rights".decode('utf-8')
                elif lng == "fr":
                    return "Droits".decode('utf-8')
                else:
                    return "Rights".decode('utf-8')
                
            def select_access_rights(self, lng="en"):
                if lng == "en":
                    return "Select access rights".decode('utf-8')
                elif lng == "fr":
                    return "Sélectionner un droits accès".decode('utf-8')
                else:
                    return "Select access rights".decode('utf-8')
                
            def select_user(self, lng="en"):
                if lng == "en":
                    return "Select user".decode('utf-8')
                elif lng == "fr":
                    return "Sélectionner un utilisateur".decode('utf-8')
                else:
                    return "Select user".decode('utf-8')
                
            def settings(self, lng="en"):
                if lng == "en":
                    return "Settings".decode('utf-8')
                elif lng == "fr":
                    return "Paramètres".decode('utf-8')
                else:
                    return "Settings".decode('utf-8')
                
            def fields_not_filled(self, lng="en"):
                if lng == "en":
                    return "Some fields are not filled".decode('utf-8')
                elif lng == "fr":
                    return "Certains champs ne sont pas remplis".decode('utf-8')
                else:
                    return "Some fields are not filled".decode('utf-8')
                
            def start(self, lng="en"):
                if lng == "en":
                    return "Start".decode('utf-8')
                elif lng == "fr":
                    return "Démarrer".decode('utf-8')
                else:
                    return "Start".decode('utf-8')
                
            def status(self, lng="en"):
                if lng == "en":
                    return "Status".decode('utf-8')
                elif lng == "fr":
                    return "Statut".decode('utf-8')
                else:
                    return "Status".decode('utf-8')
                
            def stop(self, lng="en"):
                if lng == "en":
                    return "Stop".decode('utf-8')
                elif lng == "fr":
                    return "Arrêter".decode('utf-8')
                else:
                    return "Stop".decode('utf-8')
                
            def passwords_different(self, lng="en"):
                if lng == "en":
                    return "The passwords are different".decode('utf-8')
                elif lng == "fr":
                    return "Les mots de passe sont différents".decode('utf-8')
                else:
                    return "The passwords are different".decode('utf-8')
                
            def add_user_success(self, lng="en"):
                if lng == "en":
                    return "The user was added successfully".decode('utf-8')
                elif lng == "fr":
                    return "L'utilisateur a été ajouté avec succès".decode('utf-8')
                else:
                    return "The user was added successfully".decode('utf-8')
                
            def user_mv_success(self, lng="en"):
                if lng == "en":
                    return "The user was modified successfully".decode('utf-8')
                elif lng == "fr":
                    return "L'utilisateur a été modifié avec succès".decode('utf-8')
                else:
                    return "The user was modified successfully".decode('utf-8')
                
            def user_rm_success(self, lng="en"):
                if lng == "en":
                    return "The user was removed successfully".decode('utf-8')
                elif lng == "fr":
                    return "L'utilisateur a été supprimé avec succès".decode('utf-8')
                else:
                    return "The user was removed successfully".decode('utf-8')
                
            def time(self, lng="en"):
                if lng == "en":
                    return "Time".decode('utf-8')
                elif lng == "fr":
                    return "Temps".decode('utf-8')
                else:
                    return "Time".decode('utf-8')
                
            def user(self, lng="en"):
                if lng == "en":
                    return "User".decode('utf-8')
                elif lng == "fr":
                    return "Utilisateur".decode('utf-8')
                else:
                    return "User".decode('utf-8')
                
            def user_account(self, lng="en"):
                if lng == "en":
                    return "User account".decode('utf-8')
                elif lng == "fr":
                    return "Compte utilisateur".decode('utf-8')
                else:
                    return "User account".decode('utf-8')
                
            def user_guide(self, lng="en"):
                if lng == "en":
                    return "User guide".decode('utf-8')
                elif lng == "fr":
                    return "Guide utilisateur".decode('utf-8')
                else:
                    return "User guide".decode('utf-8')
                
            def user_id(self, lng="en"):
                if lng == "en":
                    return "User id".decode('utf-8')
                elif lng == "fr":
                    return "Identifiant".decode('utf-8')
                else:
                    return "User id".decode('utf-8')
                
            def user_list(self, lng="en"):
                if lng == "en":
                    return "User list".decode('utf-8')
                elif lng == "fr":
                    return "Liste des utilisateurs".decode('utf-8')
                else:
                    return "User list".decode('utf-8')
                
            def traductor(self, lng="en"):
                if lng == "en":
                    return "Traductor".decode('utf-8')
                elif lng == "fr":
                    return "Traducteur".decode('utf-8')
                else:
                    return "Traductor".decode('utf-8')
                
            def language(self, lng="en"):
                if lng == "en":
                    return "Language".decode('utf-8')
                elif lng == "fr":
                    return "Langue".decode('utf-8')
                else:
                    return "Language".decode('utf-8')
                
        class CobotGuiStyles():
            def __init__(self):
                self.uuid = self.__class__.__name__
                self.alarm = "QWidget{background-color: #ffff01;}"
                self.background_estop_locked = "QWidget{background-color: #ff0000;}"
                self.background_estop_unlocked = "QWidget{background-color: #d9d9d9;}"
                self.login = "QDialog{background-color:qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #616763, stop: 1 #89928c);}"
                self.default_launch = "QPushButton{background-color: rgba(255,0,0,80%);border-radius: 10px;font-size: 12pt;font-weight:60;color: #ffffff;}"
                self.transparent_background = "background-color: transparent;font-size: 20pt;"
                self.no_background = "background:none;"
                self.bad_password = "background-color: #ffffff;border-radius: 5px;font-size: 16pt; font-weight:40; color: rgb(255,0,0);"
                self.good_password = "background-color: #ffffff;border-radius: 5px;font-size: 16pt; font-weight:40; color: rgb(0,255,0);"
                self.no_password = "background-color: #ffffff;border-radius: 5px;font-size: 16pt; font-weight:40; color: #494842;"
                self.text = "QLabel {font-size: 22pt;}"
            def findById(self, id=""):
                try:
                    return getattr(self,id)
                except:
                    return None
        self.uuid = self.__class__.__name__
        self.dir = DIR_COBOTGUI_VALUES
        self.launchers = DIR_COBOTGUI_VALUES+'/launchers.xml'
        self.strings = CobotGuiStrings()
        self.styles = CobotGuiStyles()
    def findById(self, id=""):
        try:
            return getattr(self,id)
        except:
            return None

class R:
    DIR = DIR_COBOTGUI_RESOURCES
    accounts = CobotGuiAccounts()
    images = CobotGuiImages()
    layouts = CobotGuiLayouts()
    values = CobotGuiValues()
    @staticmethod
    def getPixmapById(id=""):
        return QPixmap(R.images.findById(id))
    @staticmethod
    def getIconById(id=""):
        return QIcon(R.images.findById(id))


# End of file