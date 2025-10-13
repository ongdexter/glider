# Glider

Glider is a G-INS system built on [GTSAM](https://github.com/borglab/gtsam). It currently takes in GPS and 9-DOF IMU and provides a full
state estimate up to the rate of you IMU. Glider is highly configurable and more features are coming soon. 

## Hardware Setup
You're setup needs a GPS and a 9-DOF IMU, that is an IMU that provides a full orientation. The IMU orientation should be provided in the `IMU` frame
as this is standard for robotics, but we are working on supporting the NED frame.

## ROS2 Setup
We recommend using Glider with ROS2, you can configure the ros parameters in `config/ros-params.yaml`. Here's more detail about what
the parameters mean:
 - `publishers.rate`: the rate at which odometry is published in hz.
 - `publishers.nav_sat_fix`: if true will publish the odometry as a `NavSatFix` msg, the default is an `Odometry` msg.
 - `publishers.viz.use`: if true will publish an `Odometry` topic for visualization centered around the origin.
 - `publishers.viz.origin_easting`: the easting value you want to viz odometry to center around.
 - `publishers.viz.origin_northing`: the northing value you want the viz odometry to center around.
 - `subscribers.use_odom`: Still under development

## Glider Setup

### Building and Running Unit Tests
We use GTest to run unit tests. You can build the tests with 
``` 
cmake -S . -B build
cmake --build build
```
and run with:
```
cd build 
ctest
```

