# agi_docgen
<a id="top"/> 

This repository holds a graphical user interface tool

## Contents

1. <a href="#1--execution">Execution</a>


## 1. Execution: <a id="1--execution"/> 
```
roslaunch cobot_gui default.launch config_path:=${PKG_NAME}/folder file_name:=file.conf

```

By default:
```
roslaunch cobot_gui default.launch config_path:=${cobot_gui}/config file_name:=default.conf

```

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

<a href="#top">top</a>
