# Glider
![Humble CI](https://github.com/KumarRobotics/glider/workflows/humble-ci.yml/badge.svg?branch=ros2)
![Jazzy CI](https://github.com/KumarRobotics/glider/workflows/jazzy-ci.yml/badge.svg?branch=ros2)
Glider is a G-INS system built on [GTSAM](https://github.com/borglab/gtsam). It currently takes in GPS and 9-DOF IMU and provides a full
state estimate up to the rate of you IMU. Glider is highly configurable and more features are coming soon. 

## Hardware Setup
You're setup needs a GPS and a 9-DOF IMU, that is an IMU that provides a full orientation. The IMU orientation should be provided in the `IMU` frame
as this is standard for robotics, but we are working on supporting the NED frame. We use a VectorNav VN100T IMU. It is important make sure your IMU magnetometer is calibrated, if it is not aligned correctly the heading output of glider will be incorrect.

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
You can configure glider itself in `config/glider-params.yaml`, this is where you can specify the parameters for the factor graph. Here's more detail on each parameter:
#### IMU Parameters
 - `covariances.accelerometer`: covariance of the accelerometer.
 - `covariances.gyroscope`: covariance of the gyroscope.
 - `covariances.integration`: covariance of the IMU preintegration.
 - `covariances.heading`: covariance of the IMU's magnetometer heading in radians, 0.09 radians is about 5 degrees.
 - `covariances.roll_pitch`: covariance of the roll and pitch angles in radians.
 - `covariances.bias`: covariance of the bias estimate.
 - `frame`: What frame the IMU is in, currently only support ENU but NED support is coming .
### GPS Parameters
 - `gps.covariance`: covariance of the gps position estimate.
### Other Parameters
 - `constants.gravity`: gravity in your IMU's frame.
 - `constants.bias_num_measurements`: number of IMU measurements to use to initially estimate the bias.
 - `constants.initial_num_measurements`: number of times to let the factor graph optimize before glider starts reporting odometry.
 - `logging.stdout`: output log statements to terminal in addition to the logfile
 - `optimizer.smooth`: if true the factor graph will optimize using a fixed lag smoother, otherwise it will use iSAM2.
 - `optimizer.lag_time`: period of time the fixed lag smoother should look at in seconds.
 - `gps_to_imu`: the relative transformation from your gps to your imu in the FLU frame.

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

