/*
* Jason Hughes
* July 2025
*
* Take care of all necessary transforms.
*/

#include "glider/core/glider.hpp"

namespace Glider
{

Glider::Glider(const std::string& path) 
{
    Parameters params = Parameters::Load(path);
    factor_manager_ = FactorManager(params);

    frame_ = params.frame;
    t_imu_gps_ = params.t_imu_gps;

    LOG(INFO) << "[GLIDER] Using IMU frame: " << frame_;
    LOG(INFO) << "[GLIDER] Glider initialized";
}

void Glider::addGps(int64_t timestamp, Eigen::Vector3d& gps)
{
    // transform from GPS To UTM
    Eigen::Vector3d meas = Eigen::Vector3d::Zero();
    
    double easting, northing;
    char zone[4];
    geodetics::LLtoUTM(gps(0), gps(1), northing, easting, zone);
    
    if (frame_ == "ned")
    {
        meas.head(2) << northing, easting;
    }
    else
    { 
        meas.head(2) << easting, northing;
    }
    meas(2) = gps(2);

    meas = meas + t_imu_gps_;

    factor_manager_.addGpsFactor(timestamp, meas);
}

void Glider::addImu(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d& gyro, Eigen::Vector4d& quat)
{
    if (frame_ == "ned")
    {
        //TODO what transforms need to happen here
        factor_manager_.addImuFactor(timestamp, accel, gyro, quat);
    }
    else if (frame_ == "enu")
    {
        factor_manager_.addImuFactor(timestamp, accel, gyro, quat);
    }
    else
    {
        LOG(FATAL) << "IMU Frame, not supported use ENU or NED";
    }
}  

Odometry Glider::interpolate(int64_t timestamp)
{
    Odometry odom = factor_manager_.predict(timestamp);
    return odom;
}

State Glider::optimize()
{
    State state = factor_manager_.runner();
    return state;
}
}
