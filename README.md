# Topology-based Path-Planning

* HARRT*
* TARRT*

Dependencies
* libxml2
* cgal

Build
```
$ mkdir buld
$ cd build
$ cmake ..
$ make
```

Run HARRT*
```
$ bin/tpp-harrts-demo
```

Run TARRT*
```
$ bin/tpp-tarrts-demo
```
STEPS:
* load a map "Edit->Load Map"
* set start position and goal position by mouse right-click
* configure the objective and parameters "Edit->Config Objective"
* run the planner "Edit->Run"
* after planning is finished, Left and Right keys to switch different paths

TIPS
* save a plan configuration "File->Save"
* load a plan configuration "File->Open"
* save paths and data to files "File->Export"

* snapshot the GUI "Tool->Save Screen"
