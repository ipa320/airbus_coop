^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ssm_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2017-09-05)
------------------
* Changed error display to give the name of the state and the name of the problem
  Fixing a test if an attribute is given without is expr set.
* add the feedback to the action server
* changed some error messages
* created a launch for the action server
* fixed the cmakelists files and removed the executable file in the source
* added an action server for scxml execution
* added a test to make both way working
* moved the empty state to the ssm_state file
* re-organized the main function to be a bit more clear
* Added license files
* Changed order in the the licensing header
* added Airbus to the licensing header
* added licensing apache2.0 header
* update and anify package.xml files
* unnecessary re-declaration of param. gui_plugin should still work.
* changed the launch file and the node / main to get the argument for the file you want to use
* fixed a named changed
* removed last reference to pyqt_agi_extend
* some changes to set the default log path to be at /tmp/ and removed the ssm_log dir
* removed a dependacy to pyqt_agi_extend
* changed the skill provider so in doesn't depend on pyqt_agi_extend lib
* fixed launch files
* fixed setup and CMakeLists / package.xml
* removed executable parts of main and interpreter and tranfered it to a node file (good practice).
* add install for ssm_main
* added the readme on ssm_core
* added the parameter to the launchs files
* changed the syntax of the test
* added a "ssm_enable_log" parameter to enable or disable the log
* changed the default path for interpreter and main
* changed the launch file to the default scxml
* created a default scxml file and a empty skill register for test purposes
* moved test skills and test.scxml from core to tutorial
* Removed all hardware specific plugins and dashboard, cleaning up all the hardware specifics launchs / configs files
* Review CMakelists and package files
* added needed dependencies packages
* Contributors: ipa-led, ipa-nhg
