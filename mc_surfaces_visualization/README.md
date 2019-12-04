# mc\_surfaces\_visualization

This tool allows to visualize the surfaces of an mc\_rtc robot in the RViZ application.

## Launching the visualization

The visualization is launched as follows:

```bash
$ roslaunch mc_surfaces_visualization display.launch robot:=JVRC1
```

The `robot` argument should be the same as what you would pass to the `mc_rbdyn::RobotLoader::get_robot_module` function, this can also be a robot alias. For example:

```bash
# Specify a vector of arguments
$ roslaunch mc_surfaces_visualization display.launch robot:="[env, `rospack find mc_env_description`, ground]"
# Or an alias
$ roslaunch mc_surfaces_visualization display.launch robot:=env/ground
```

## Surfaces visualization

In the visualization:

- planar surfaces are drawn in green with a blue arrow showing the normal direction of the surface
- cylindrical surfaces are green cylinders
- gripper surfaces are represented with blue arrows representing the normal direction of the gripper's points' frames

You can show/hide selected surfaces by enabling/disabling their namespaces in the associated MarkerArray
