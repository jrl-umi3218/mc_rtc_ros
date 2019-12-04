# mc\_convex\_visualization

This tool allows to visualize the convexes of an mc\_rtc robot in the RViZ application.

## Launching the visualization

The visualization is launched as follows:

```bash
$ roslaunch mc_convex_visualization display.launch robot:=JVRC1
```

The `robot` argument should be the same as what you would pass to the `mc_rbdyn::RobotLoader::get_robot_module` function, this can also be a robot alias. For example:

```bash
# Specify a vector of arguments
$ roslaunch mc_convex_visualization display.launch robot:="[env, `rospack find mc_env_description`, ground]"
# Or an alias
$ roslaunch mc_convex_visualization display.launch robot:=env/ground
```

## Convexes visualization

You can show/hide selected convexes by enabling/disabling their namespaces in the associated MarkerArray
