^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package airbus_cobot_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.4 (2017-09-13)
------------------
* Merge pull request `#36 <https://github.com/ipa320/airbus_coop/issues/36>`_ from ipa-led/master
  Added dotgraph viewer to ssm_plugin
* removed a debug print
* Merge pull request `#35 <https://github.com/ipa320/airbus_coop/issues/35>`_ from ipa-led/master
  Improvements on the ssm_core packages
* removed templates from the config file
* fixed default view set to rviz
* Contributors: Nadia Hammoudeh García, ipa-led

0.0.3 (2017-09-06)
------------------
* Merge pull request `#34 <https://github.com/ipa320/airbus_coop/issues/34>`_ from ipa-nhg/SeparateTemplates
  Separate templates
* make airbus_cobot_gui independet to the templates
* Merge pull request `#33 <https://github.com/ipa320/airbus_coop/issues/33>`_ from ipa-nhg/Rename
  Rename packages
* test cobot_gui renames
* renamed packages
* rename plugin_rviz to airbus_plugin_rviz
* rename plugin_rqt to airbus_plugin_rqt
* rename plugin_node_manager to airbus_plugin_node_manager
* renamed plugin_log_manager to airbus_plugin_log_manager
* rename pyqt_agi_extend to airbus_pyqt_extend
* rename cobot_gui to airbus_cobot_gui
* Contributors: Nadia Hammoudeh García, ipa-nhg

0.0.2 (2017-09-05)
------------------
* Diagnostics tool integrated
* update the diagnostic plugin adding signals
* minor fixes
* implement the diagnostics subscriber
* update the diagnostics.ui
* add the topics names as input parameters
* add diagnoctics popup class
* added tiptool
* add standard diagnostics to the airbus_cobot_gui
* setup signal so the closing is working with rosnode kill command
* Add Error print when creating / destroying object
* changed the code to solve launching / killing problem
* ctrl+c should now stop the gui and close every windows
* fixed encoding in resources files
* Changed the default config and the register to show the template dashboard and plugin
* Added license files
* Changed order in the the licensing header
* added Airbus to the licensing header
* added licensing apache2.0 header
* update and anify package.xml files
* update readme for airbus_cobot_gui
* Changed explanation on group again
* Change the explanation on group
* changes some words and fixed the content
  Signed-off-by: ipa-led <ludovic.delval@ipa.fraunhofer.de>
* added explanation on the config file
* fixed the launch file
* added readme for airbus_cobot_gui
* add arguments to launch file
* Fixing deskop-launch with the working launcher
* Removed the old main script from the CMakeLists.txt
* Removed the very old version of the gui launcher and the ui
* Fixing airbus_cobot_gui_node by changing the path to the GUI configuration file
* Removed all hardware specific plugins and dashboard, cleaning up all the hardware specifics launchs / configs files
* Review CMakelists and package files
* test install version
* fix setup.py files
* update install tags
* removed hardware especific packages
* added needed dependencies packages
* Contributors: ipa-led, ipa-nhg
