This package loads all surfaces of a given robot and publishes
them as markers into RViz.

Usage:
This package depends on ROS. To build, place yourself in a
catkin_workspace and simply run catkin_make.

```
git clone --recursive git@gite.lirmm.fr:multi-contact/mc_surfaces_visualization_ros.git
roscd
catkin_make
(optional) catkin_make install
```

Requires:
```bash
#Non ros
mc_rtc
boost (program_options)
#ROS
geometry_msgs
visualization_msgs
```

Once built, just run:
```
roslaunch surfaces_visualization_ros display.launch
```

To change the robot, one has to specify:
- The package
- The robot

To change the package, you *must* edit `display.launch` because ROS pre-Kinetic
does not allow recursive argument substitution. To change the robot, simply use
the `robot` argument like so:
```bash
roslaunch surfaces_visualization_ros display.launch robot:=my_robot
```

For example,
```bash
roslaunch surfaces_visualization_ros display.launch robot:=litter
```

Note that if the robot does not display correctly, it is probably because it does
not have a `base_link`. In that case, simply change the fixed reference frame to
any frame in Rviz.
