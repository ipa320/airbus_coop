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

import sys

# from qt_gui.main import Main as Base
import os
import rospy
from rospkg.rospack import RosPack

from rqt_gui.main import Main
from rqt_gui.main import Base

from argparse import ArgumentParser, SUPPRESS
import platform
import signal
import sys

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *

from cobot_gui import Plugin

class RqtGuiWrapperClass(Main):
    
    def __init__(self, filename=None, ros_pack=None):
        rp = ros_pack or RosPack()
        qtgui_path = rp.get_path('qt_gui')
        super(Main, self).__init__(qtgui_path, invoked_filename=filename, settings_filename='rqt_gui')
        self._ros_pack = rp
    
    def main(self, argv=None, standalone=None, plugin_argument_provider=None):
        if argv is None:
            argv = sys.argv
        
        # extract --args and everything behind manually since argparse can not handle that
        arguments = argv[1:]
        
        # extract plugin specific args when not being invoked in standalone mode programmatically
        if not standalone:
            plugin_args = []
            if '--args' in arguments:
                index = arguments.index('--args')
                plugin_args = arguments[index + 1:]
                arguments = arguments[0:index + 1]
        
        parser = ArgumentParser(os.path.basename(Main.main_filename),
                                add_help=False)
        self.add_arguments(parser, standalone=bool(standalone),
                           plugin_argument_provider=plugin_argument_provider)
        self._options = parser.parse_args(arguments)
        
        if standalone:
            # rerun parsing to separate common arguments from plugin specific arguments
            parser = ArgumentParser(os.path.basename(Main.main_filename),
                                    add_help=False)
            self.add_arguments(parser, standalone=bool(standalone))
            self._options, plugin_args = parser.parse_known_args(arguments)
        self._options.plugin_args = plugin_args
        
        # set default values for options not available in standalone mode
        if standalone:
            self._options.freeze_layout = False
            self._options.lock_perspective = False
            self._options.multi_process = False
            self._options.perspective = None
            self._options.perspective_file = None
            self._options.standalone_plugin = standalone
            self._options.list_perspectives = False
            self._options.list_plugins = False
            self._options.command_pid = None
            self._options.command_start_plugin = None
            self._options.command_switch_perspective = None
            self._options.embed_plugin = None
            self._options.embed_plugin_serial = None
            self._options.embed_plugin_address = None
        
        # check option dependencies
        try:
            if self._options.plugin_args and \
            not self._options.standalone_plugin and \
            not self._options.command_start_plugin and \
            not self._options.embed_plugin:
                raise RuntimeError('Option --args can only be used together with either --standalone, --command-start-plugin or --embed-plugin option')
            
            if self._options.freeze_layout and \
               not self._options.lock_perspective:
                raise RuntimeError('Option --freeze_layout can only be used together with the --lock_perspective option')
            
            list_options = (self._options.list_perspectives,
                            self._options.list_plugins)
            list_options_set = [opt for opt in list_options if opt is not False]
            if len(list_options_set) > 1:
                raise RuntimeError('Only one --list-* option can be used at a time')
            
            command_options = (self._options.command_start_plugin,
                               self._options.command_switch_perspective)
            command_options_set = [opt for opt in command_options if opt is not None]
            if len(command_options_set) > 0 and not self._dbus_available:
                raise RuntimeError('Without DBus support the --command-* options are not available')
            if len(command_options_set) > 1:
                raise RuntimeError('Only one --command-* option can be used at a time (except --command-pid which is optional)')
            if len(command_options_set) == 0 and \
               self._options.command_pid is not None:
                raise RuntimeError('Option --command_pid can only be used together with an other --command-* option')
            
            embed_options = (self._options.embed_plugin,
                             self._options.embed_plugin_serial,
                             self._options.embed_plugin_address)
            embed_options_set = [opt for opt in embed_options if opt is not None]
            if len(command_options_set) > 0 and not self._dbus_available:
                raise RuntimeError('Without DBus support the --embed-* options are not available')
            if len(embed_options_set) > 0 and \
               len(embed_options_set) < len(embed_options):
                raise RuntimeError('Missing option(s) - all \'--embed-*\' options must be set')
            
            if len(embed_options_set) > 0 and self._options.clear_config:
                raise RuntimeError('Option --clear-config can only be used without any --embed-* option')
            
            groups = (list_options_set, command_options_set, embed_options_set)
            groups_set = [opt for opt in groups if len(opt) > 0]
            if len(groups_set) > 1:
                raise RuntimeError('Options from different groups (--list, --command, --embed) can not be used together')
            
            perspective_options = (self._options.perspective,
                                   self._options.perspective_file)
            perspective_options_set = [opt for opt in perspective_options if opt is not None]
            if len(perspective_options_set) > 1:
                raise RuntimeError('Only one --perspective-* option can be used at a time')
            
            if self._options.perspective_file is not None and \
               not os.path.isfile(self._options.perspective_file):
                raise RuntimeError('Option --perspective-file must reference existing file')
        
        except RuntimeError as e:
            print(str(e))
            #parser.parse_args(['--help'])
            # calling --help will exit
            return 1
        
        # set implicit option dependencies
        if self._options.standalone_plugin is not None:
            self._options.lock_perspective = True
        
        # create application context containing various relevant information
        from qt_gui.application_context import ApplicationContext
        context = ApplicationContext()
        context.qtgui_path = self._qtgui_path
        context.options = self._options
        
        if self._dbus_available:
            from dbus import DBusException, Interface, SessionBus
        
        # non-special applications provide various dbus interfaces
        if self._dbus_available:
            context.provide_app_dbus_interfaces = len(groups_set) == 0
            context.dbus_base_bus_name = 'org.ros.qt_gui'
            if context.provide_app_dbus_interfaces:
                context.dbus_unique_bus_name = context.dbus_base_bus_name + '.pid%d' % os.getpid()
                
                # provide pid of application via dbus
                from qt_gui.application_dbus_interface import ApplicationDBusInterface
                _dbus_server = ApplicationDBusInterface(context.dbus_base_bus_name)
        
        # determine host bus name, either based on pid given on command line or via dbus application interface if any other instance is available
        if len(command_options_set) > 0 or len(embed_options_set) > 0:
            host_pid = None
            if self._options.command_pid is not None:
                host_pid = self._options.command_pid
            else:
                try:
                    remote_object = SessionBus().get_object(context.dbus_base_bus_name,
                                                            '/Application')
                except DBusException:
                    pass
                else:
                    remote_interface = Interface(remote_object,
                                                 context.dbus_base_bus_name + '.Application')
                    host_pid = remote_interface.get_pid()
            if host_pid is not None:
                context.dbus_host_bus_name = context.dbus_base_bus_name + '.pid%d' % host_pid
        
        # execute command on host application instance
        if len(command_options_set) > 0:
            if self._options.command_start_plugin is not None:
                try:
                    remote_object = SessionBus().get_object(context.dbus_host_bus_name,
                                                            '/PluginManager')
                except DBusException:
                    (rc, msg) = (1, 
                                 'unable to communicate with GUI instance "%s"'
                                 % context.dbus_host_bus_name)
                else:
                    remote_interface = Interface(remote_object,
                                                 context.dbus_base_bus_name + '.PluginManager')
                    (rc, msg) = remote_interface.start_plugin(self._options.command_start_plugin,
                                                              ' '.join(self._options.plugin_args))
                if rc == 0:
                    print('qt_gui_main() started plugin "%s" in GUI "%s"'
                          %(msg, context.dbus_host_bus_name))
                else:
                    print('qt_gui_main() could not start plugin "%s" in GUI "%s": %s'
                          %(self._options.command_start_plugin,
                            context.dbus_host_bus_name, msg))
                return rc
            elif self._options.command_switch_perspective is not None:
                remote_object = SessionBus().get_object(context.dbus_host_bus_name,
                                                        '/PerspectiveManager')
                remote_interface = Interface(remote_object,
                                             context.dbus_base_bus_name+'.PerspectiveManager')
                remote_interface.switch_perspective(self._options.command_switch_perspective)
                print('qt_gui_main() switched to perspective "%s" in GUI "%s"'
                      %(self._options.command_switch_perspective,
                        context.dbus_host_bus_name))
                return 0
            raise RuntimeError('Unknown command not handled')
        
        # choose selected or default qt binding
        setattr(sys, 'SELECT_QT_BINDING', self._options.qt_binding)
        from python_qt_binding import QT_BINDING
        
        from python_qt_binding.QtCore import qDebug, qInstallMsgHandler, \
                                             QSettings, Qt, QtCriticalMsg, \
                                             QtDebugMsg, QtFatalMsg, \
                                             QTimer, QtWarningMsg
        from python_qt_binding.QtGui import QAction, QIcon, QMenuBar
        
        from qt_gui.about_handler import AboutHandler
        from qt_gui.composite_plugin_provider import CompositePluginProvider
        from qt_gui.container_manager import ContainerManager
        from qt_gui.help_provider import HelpProvider
        from qt_gui.main_window import MainWindow
        from qt_gui.minimized_dock_widgets_toolbar import MinimizedDockWidgetsToolbar
        from qt_gui.perspective_manager import PerspectiveManager
        from qt_gui.plugin_manager import PluginManager
        
        def message_handler(type_, msg):
            colored_output = 'TERM' in os.environ and 'ANSI_COLORS_DISABLED' not in os.environ
            cyan_color = '\033[36m' if colored_output else ''
            red_color = '\033[31m' if colored_output else ''
            reset_color = '\033[0m' if colored_output else ''
            if type_ == QtDebugMsg and self._options.verbose:
                print(msg, sys.stderr)
            elif type_ == QtWarningMsg:
                print(cyan_color + msg + reset_color, sys.stderr)
            elif type_ == QtCriticalMsg:
                print(red_color + msg + reset_color, sys.stderr)
            elif type_ == QtFatalMsg:
                print(red_color + msg + reset_color, sys.stderr)
                sys.exit(1)
                
        qInstallMsgHandler(message_handler)
        
        app = self.create_application(argv)
        
        self._check_icon_theme_compliance()
        
        settings = QSettings(QSettings.IniFormat,
                             QSettings.UserScope,
                             'ros.org',
                             self._settings_filename)
        if len(embed_options_set) == 0:
            if self._options.clear_config:
                settings.clear()
            
            main_window = MainWindow()
            
            if self._options.on_top:
                main_window.setWindowFlags(Qt.WindowStaysOnTopHint)
            
            main_window.statusBar()
            
            def sigint_handler(*args):
                qDebug('\nsigint_handler()')
                main_window.close()
            signal.signal(signal.SIGINT, sigint_handler)
            # the timer enables triggering the sigint_handler
            timer = QTimer()
            timer.start(500)
            timer.timeout.connect(lambda: None)
            
            # create own menu bar to share one menu bar on Mac
            menu_bar = QMenuBar()
            if 'darwin' in platform.platform().lower():
                menu_bar.setNativeMenuBar(True)
            else:
                menu_bar.setNativeMenuBar(False)
            if not self._options.lock_perspective:
                main_window.setMenuBar(menu_bar)
            
            file_menu = menu_bar.addMenu(menu_bar.tr('File'))
            action = QAction(file_menu.tr('Quit'), file_menu)
            action.setIcon(QIcon.fromTheme('application-exit'))
            action.triggered.connect(main_window.close)
            file_menu.addAction(action)
            
        else:
            app.setQuitOnLastWindowClosed(False)
            
            main_window = None
            menu_bar = None
        
        self._add_plugin_providers()
        
        # setup plugin manager
        plugin_provider = CompositePluginProvider(self.plugin_providers)
        plugin_manager = PluginManager(plugin_provider, settings, context)
        
        if self._options.list_plugins:
            # output available plugins
            print('\n'.join(sorted(plugin_manager.get_plugins().values())))
            return 0
        
        help_provider = HelpProvider()
        plugin_manager.plugin_help_signal.connect(help_provider.plugin_help_request)
        
        # setup perspective manager
        if main_window is not None:
            perspective_manager = PerspectiveManager(settings, context)
        
            if self._options.list_perspectives:
                # output available perspectives
                print('\n'.join(sorted(perspective_manager.perspectives)))
                return 0
        else:
            perspective_manager = None
            
        if main_window is not None:
            container_manager = ContainerManager(main_window, plugin_manager)
            plugin_manager.set_main_window(main_window, menu_bar, container_manager)

            if not self._options.freeze_layout:
                minimized_dock_widgets_toolbar = MinimizedDockWidgetsToolbar(container_manager, main_window)
                main_window.addToolBar(Qt.BottomToolBarArea, minimized_dock_widgets_toolbar)
                plugin_manager.set_minimized_dock_widgets_toolbar(minimized_dock_widgets_toolbar)

        if menu_bar is not None:
            perspective_menu = menu_bar.addMenu(menu_bar.tr('P&erspectives'))
            perspective_manager.set_menu(perspective_menu)
            
        
        # connect various signals and slots
        if perspective_manager is not None and main_window is not None:
            # signal changed perspective to update window title
            perspective_manager.perspective_changed_signal.connect(main_window.perspective_changed)
            # signal new settings due to changed perspective
            perspective_manager.save_settings_signal.connect(main_window.save_settings)
            perspective_manager.restore_settings_signal.connect(main_window.restore_settings)
            perspective_manager.restore_settings_without_plugin_changes_signal.connect(main_window.restore_settings)
        
        if perspective_manager is not None and plugin_manager is not None:
            perspective_manager.save_settings_signal.connect(plugin_manager.save_settings)
            plugin_manager.save_settings_completed_signal.connect(perspective_manager.save_settings_completed)
            perspective_manager.restore_settings_signal.connect(plugin_manager.restore_settings)
            perspective_manager.restore_settings_without_plugin_changes_signal.connect(plugin_manager.restore_settings_without_plugins)
        
        if plugin_manager is not None and main_window is not None:
            # signal before changing plugins to save window state
            plugin_manager.plugins_about_to_change_signal.connect(main_window.save_setup)
            # signal changed plugins to restore window state
            plugin_manager.plugins_changed_signal.connect(main_window.restore_state)
            # signal save settings to store plugin setup on close
            main_window.save_settings_before_close_signal.connect(plugin_manager.close_application)
            # signal save and shutdown called for all plugins, trigger closing main window again
            plugin_manager.close_application_signal.connect(main_window.close,
                                                            type=Qt.QueuedConnection)
        
        if main_window is not None and menu_bar is not None:
            about_handler = AboutHandler(context.qtgui_path, main_window)
            help_menu = menu_bar.addMenu(menu_bar.tr('Help'))
            action = QAction(file_menu.tr('About'), help_menu)
            action.setIcon(QIcon.fromTheme('help-about'))
            action.triggered.connect(about_handler.show)
            help_menu.addAction(action)
        
        # set initial size - only used without saved configuration
        if main_window is not None:
            main_window.resize(600, 450)
            main_window.move(100, 100)
        
        # ensure that qt_gui/src is in sys.path
        src_path = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
        if src_path not in sys.path:
            sys.path.append(src_path)
        
        # load specific plugin
        plugin = None
        plugin_serial = None
        if self._options.embed_plugin is not None:
            plugin = self._options.embed_plugin
            plugin_serial = self._options.embed_plugin_serial
        elif self._options.standalone_plugin is not None:
            plugin = self._options.standalone_plugin
            plugin_serial = 0
        if plugin is not None:
            plugins = plugin_manager.find_plugins_by_name(plugin)
            if len(plugins) == 0:
                print('qt_gui_main() found no plugin matching "%s"' % plugin)
                return 1
            elif len(plugins) > 1:
                print('qt_gui_main() found multiple plugins matching "%s"\n%s'
                      %(plugin, '\n'.join(plugins.values())))
                return 1
            plugin = plugins.keys()[0]
        
        qDebug('QtBindingHelper using %s' % QT_BINDING)
        
        plugin_manager.discover()
        
        if self._options.reload_import:
            qDebug('ReloadImporter() automatically reload all subsequent imports')
            from .reload_importer import ReloadImporter
            _reload_importer = ReloadImporter()
            self._add_reload_paths(_reload_importer)
            _reload_importer.enable()
        
        # switch perspective
        if perspective_manager is not None:
            if plugin:
                perspective_manager.set_perspective(plugin,
                                                    hide_and_without_plugin_changes=True)
            elif self._options.perspective_file:
                perspective_manager.import_perspective_from_file(self._options.perspective_file,
                                                                 perspective_manager.HIDDEN_PREFIX + '__cli_perspective_from_file')
            else:
                perspective_manager.set_perspective(self._options.perspective)
        
        # load specific plugin
        if plugin:
            plugin_manager.load_plugin(plugin, plugin_serial,
                                       self._options.plugin_args)
            running = plugin_manager.is_plugin_running(plugin, plugin_serial)
            if not running:
                return 1
        
        return main_window
    
    def create_application(self, argv):
        pass
    
    def _add_plugin_providers(self):
        # do not import earlier as it would import Qt stuff without the proper initialization from qt_gui.main
        from qt_gui.recursive_plugin_provider import RecursivePluginProvider
        from rqt_gui.rospkg_plugin_provider import RospkgPluginProvider
        RospkgPluginProvider.rospack = self._ros_pack
        self.plugin_providers.append(RospkgPluginProvider('qt_gui',
                                                          'qt_gui_py::Plugin'))
        self.plugin_providers.append(RecursivePluginProvider(RospkgPluginProvider('qt_gui',
                                                                                  'qt_gui_py::PluginProvider')))
        self.plugin_providers.append(RecursivePluginProvider(RospkgPluginProvider('rqt_gui',
                                                                                  'rqt_gui_py::PluginProvider')))
    
    def _add_reload_paths(self, reload_importer):
        super(Main, self)._add_reload_paths(reload_importer)
        reload_importer.add_reload_path(os.path.join(os.path.dirname(__file__),
                                                     *('..',) * 4))

class PluginRqt(Plugin):
     
    def __init__(self, context):
        Plugin.__init__(self, context)
        
        self._wrapper_rqt = None
        
    def onCreate(self, param):
        
        layout = QGridLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        self._wrapper_rqt = RqtGuiWrapperClass()
        
        layout.addWidget(self._wrapper_rqt.main(), 0, 0, 0, 0)
        
    def onPause(self):
        pass
    
    def onResume(self):
        pass
    
    def onControlModeChanged(self, mode):
        pass
        
    def onUserChanged(self, user_info):
        pass
    
    def onTranslate(self, lng):
        pass
    
    def onEmergencyStop(self, state):
        pass
    
    def onDestroy(self):
        pass
    
    
#End of file
