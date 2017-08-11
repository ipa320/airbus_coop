# agi_docgen
<a id="top"/> 

This repository holds a graphical user interface tool

## Contents

1. <a href="#1--execution">Execution</a>
2. <a href="#2--configuration">Configuration</a>


## 1. Execution: <a id="1--execution"/> 
```
roslaunch cobot_gui default.launch config_path:=${PKG_NAME}/folder file_name:=file.conf

```

By default:
```
roslaunch cobot_gui default.launch config_path:=${cobot_gui}/config file_name:=default.conf

```


## 2. Configuration: <a id="2--configuration"/> 

Example of configuration file:

```
<?xml version="1.0"?>

<app mode="debug">
    <translate type="en"/>
    <window display-mode="-d">
        <default-size>
            <width>1280</width>
            <height>720</height>
        </default-size>
        <header>
            <dashboards src="${cobot_gui}/config/default_dashboards_register.xml">
               <dashboard name="ExampleDashboard"/>
            </dashboards>
        </header>
        <launcher default-view="Rviz" default-mode="manu">
            <plugins src="${cobot_gui}/config/default_plugins_register.xml">
                <plugin name="Rviz"/>
                <plugin name="Rqt"/>
                <group name="Monitoring" icon="${cobot_gui}/resources/images/icon_monitoring.png">
                    <plugin name="NodeManager"/>
                    <plugin name="LogManager"/>
                </group>
           </plugins>
        </launcher>
    </window>
</app>

```

**Line by line explanation :**

```
<app mode="debug">
   ...
</app>
```
Set the mode to 'debug' or 'release'. This may be usefull when you want to get some informations only in debug mode.



```
<translate type="en"/>
```
The default language of you GUI. It can be set to 'en', 'fr', 'de' or 'es'.
The function "onTranslate" of every plugins is called when you change the language in the GUI.
You may create a xml file like this one :
```
<resources>
    <strings default-lng="en">
        <string id="hello">
            <en>Hello world !</en>
            <fr>Bonjour le monde !</fr>
            <de>Hallo Welt !</de>
            <es>Hola Mundo !</es>
        </string>
    </strings>
</resources>
```
Ane call the QLabel "setText" function with the language in parameter in the "onTranslate"



```
<window display-mode="-d">
    <default-size>
       <width>1280</width>
       <height>720</height>
    </default-size>
    ...
</window>
```
The display mode can be set to either '-d' wihch is default size then you have to specified the size, or '-f' which is full size.



```
<header>
   <dashboards src="${cobot_gui}/config/default_dashboards_register.xml">
      <dashboard name="ExampleDashboard"/>
   </dashboards>
</header>
```

This is used to set dashboard on the header of the gui. In the 'src' you specified the path to the dashboards register xml file.
Then you can pick which dashboard(s) you want to use by calling it(them).



```
<launcher default-view="Rviz" default-mode="manu">
     <plugins src="${cobot_gui}/config/default_plugins_register.xml">
        <plugin name="Rviz"/>
        <plugin name="Rqt"/>
        <group name="Monitoring" icon="${cobot_gui}/resources/images/icon_monitoring.png">
           <plugin name="NodeManager"/>
           <plugin name="LogManager"/>
        </group>
     </plugins>
</launcher>
```

In the launcher, you choose which plugins you want to launch.
You can set in 'default-view' the plugin name you want to start on and in 'default-mode' if you want to start in manual('manu') or automatic('auto'). Some buttons can be hidden / not work in some mode depending on the plugin.
In the 'src' you enter the path to the plugins register xml file then you list the plugins you want to use. **The order matter !** . It will be in this order that your plugins are gonna be displayed.
If you want to group multiple plugins under one button, you can use "group" and set a path to an icon.





<a href="#top">top</a>
