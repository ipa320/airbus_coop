# airbus_docgen
<a id="top"/> 

This repository holds a tool to autogenerate documentation, this program will generate a html documentation for a desired package or workspace

## Contents

1. <a href="#1--execution">Execution</a>


## 1. Execution: <a id="1--execution"/>

The launch file airbus_docgen.launch can be used to call the tool:

```
roslaunch airbus_docgen airbus_docgen.launch output_path:="/tmp/docu" ros_pkg:="/home/user/my_workspace"
```
or:
```
roslaunch airbus_docgen airbus_docgen.launch output_path:="/tmp/docu" ros_pkg:="package_name"
```

Input parameters are:
- output_path: the path where should be saved the generated documentation (optional, by default /tmp/docu)
- ros_pkg: the name of the package that has to be documented or the name of the workspace that contains all the packages to be documented

In case you want to document your current workspace and save the documentation in the /tmp/docu directory, you can just call the index.py node:

```
rosrun airbus_docgen index.py
```

<a href="#top">top</a>
