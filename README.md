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

