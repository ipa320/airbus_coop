# airbus_ssm_core : Smart State Machine (SSM)
<a id="top"/> 

This repository holds a SMACH overhaul including an interpretor of SCXML files.([SCXML standard](https://www.w3.org/TR/scxml/)).
It creates a finite state machine (based on SMACH) using the data on the scxml.
It can be usefull if you want to create a state machine using a GUI that generate a SCXML file like :
* [QT SCXML Editor Plugin](https://doc.qt.io/qtcreator/creator-scxml.html)
* [SCXMLGui](https://github.com/fmorbini/scxmlgui)

## Contents

1. <a href="#1--construction">Basic creation of a SCXML file</a>
2. <a href="#2--execution">Execution</a>

## 1. Basic creation of a SCXML file: <a id="1--construction"/>

This is the SCXML file from the ssm_tutorial1 

```
<?xml version="1.0" encoding="UTF-8"?>
<scxml initial="Input Number">
	<datamodel>
        <data id="skill_file" expr="${airbus_ssm_tutorial}/resources/skills.xml"/>
    </datamodel>
    <state id="Input Number">
		<datamodel>
            <data id="skill" expr="Input"/>
        </datamodel>
        <transition event="Out" target="Primes"/>
        <transition event="Test" target="IsPrime"/>
        <transition event="Retry" target="Input Number"/>
    </state>
    <state id="Primes">
		<datamodel>
            <data id="skill" expr="Primes"/>
        </datamodel>
        <transition event="Off" target="End"/>
        <transition event="Reset" target="Input Number"/>
        <transition event="Continue" target="Input Number"/>
    </state>
    <state id="IsPrime">
        <transition type="external" event="Return" target="Input Number"/>
        <datamodel>
            <data id="skill" expr="isPrime"/>
        </datamodel>
    </state>
    <final id="End"/>
</scxml>
```
If you want to learn more about the SCXML format, check : [SCXML standard](https://www.w3.org/TR/scxml/).

To link a SCXML State to a Python SSM State (an overhaul of the SMACH State) there is two things to do :

1. Give a skill register file in the datamodel at the root level.

```
<?xml version="1.0" encoding="UTF-8"?>
<scxml initial="Input Number">
	<datamodel>
        <data id="skill_file" expr="${airbus_ssm_tutorial}/resources/skills.xml"/>
    </datamodel>
    ...
</scxml>
```
The skill register file contains informations to find the Python SSM State classes you want to use inside the SCXML.

For example the skill.xml of the ssm_tutorial :
```
<?xml version="1.0"?>
<skills>
	<skill name="Input"		pkg="airbus_ssm_tutorial" 	module="airbus_ssm_tutorial1.skills" 	class="Input"/>
	<skill name="isPrime"	pkg="airbus_ssm_tutorial" 	module="airbus_ssm_tutorial1.skills" 	class="isPrime"/>
	<skill name="Primes"	pkg="airbus_ssm_tutorial" 	module="airbus_ssm_tutorial1.skills" 	class="Primes"/>
</skills>
```

**Content of a skill in the skill.xml file :**
* 'name'    : The name you want to use inside the SCXML. It can be different from the class name.
* 'pkg'     : The python package where the python file containing the class is located.
* 'module'  : The python path to the file containing the class.
* 'class'   : The class name of the SSMState.

2. Set a SSMState on your SCXML state using a data in the datanodel of the SCXML state.
```
<state id="IsPrime">
	<datamodel>
		<data id="skill" expr="isPrime"/>
	</datamodel>
	...
</state>
```
Using the 'id' "skill" and set the 'expr' value to *the name of the skill you want to use*.
You create a link between the python SSMState and the SCXML state.
This also mean that you have to link every outcome of this SSMState inside the SCXML. If you forget to link one, them the State Machine Constitency check will fail.

**To help you know every outcomes of each SSMState, you can run the 'ssm_descriptor'.**
This will scan all the skill inside a skill.xml file and will give you a text file with every outcomes and user defined data key used in each skill of the file.
```
roslaunch airbus_ssm_core ssm_descriptor.launch skill_xml_file:=empty_register.xml output_file:=/tmp/descriptor.txt
```
Input parameter are:
- skill_xml_file: the skill.xml file you want to scan. It should be located in the airbus_ssm_core/resources folder. 
If you want to put the skill.xml file in a different folder/pkg, use this syntax *${pkg_name}/directory/file.scxml*
- output_file: the absolute path for the outout text file.


## 2. Execution: <a id="2--execution"/> 

To execute a SCXML file you can use :
```
roslaunch airbus_ssm_core ssm.launch scxml_file:=default
```
Input parameter is:
- scxml_file: the scxml file you want to execute. It should be located in the airbus_ssm_core/resources folder. 
If you want to put scxml files in a different folder, use this syntax *${pkg_name}/directory/file.scxml*  or you can give the pull path to the file.

For example : 
```
roslaunch airbus_ssm_core ssm.launch scxml_file:=${airbus_ssm_core}/resources/defaulf.scxml
```
There is a service to 'load' a new SCXML file on the :
- server_name + '/srv/init' (default : '/ssm/srv/init')

You can also use topics :
- server_name + '/start' (default : '/ssm/start'), std_msgs/Empty. Trigger the execution of the state machine.
- server_name + '/preempt' (default : '/ssm/preempt'), std_msgs/Empty. Trigger the interruption (premption) of the state machine.
- server_name + '/pause' (default : '/ssm/pause'), std_msgs/Bool. If True, pause the execution of the state machine. If False, resume the execution.


To launch the SSM Simple Action Server you can use :
```
roslaunch airbus_ssm_core ssm_action_server.launch 
```

The goal are the SCXML file you want to execute using the same restriction of the input parameter scxml_file.
The result is the outcome of the last executed state.

The following topics works as well during execution :
- server_name + '/preempt' (default : '/ssm/preempt'), std_msgs/Empty. Trigger the interruption (premption) of the state machine.
- server_name + '/pause' (default : '/ssm/pause'), std_msgs/Bool. If True, pause the execution of the state machine. If False, resume the execution.


<a href="#top">top</a>

