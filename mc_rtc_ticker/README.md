# mc\_rtc\_ticker a ROS-based mc\_rtc ticker

The aim of this package is to provide a simple ROS ticker to test mc\_rtc controller code in an isolated setting.

The following ROS parameters can be defined to control the ticker:

### mc\_rtc\_ticker/conf

Provides a path to the configuration file loaded by mc\_rtc.

### mc\_rtc\_ticker/init\_pos

The initial free-flyer position as 7d vector: {qw, qx, qy, qz, tx, ty, tz}.

### mc\_rtc\_ticker/init\_state

The initial joint-state for the robot, provided in the related reference joint order.

## Visualization

This package also comes with a visualizer for mc\_rtc internal state. It is launched as follow:

```bash
$ roslaunch mc_rtc_ticker display.launch
```
