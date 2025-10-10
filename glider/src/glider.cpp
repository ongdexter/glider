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
    initializeLogging(params);
    factor_manager_ = FactorManager(params);

    frame_ = params.frame;
    t_imu_gps_ = params.t_imu_gps;

    LOG(INFO) << "[GLIDER] Using IMU frame: " << frame_;
    LOG(INFO) << "[GLIDER] Using Fixed Lag Smoother: " << std::boolalpha << params.smooth;
    LOG(INFO) << "[GLIDER] Glider initialized";
}

void Glider::initializeLogging(const Parameters& params) const
{
    google::InitGoogleLogging("Glider");
    FLAGS_log_dir = "/home/jason/.ros/log/glider";
    if (params.log) FLAGS_alsologtostderr = 1;
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

    // TODO t_imu_gps_ needs to be rotated!!
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
        LOG(FATAL) << "[GLIDER] IMU Frame, not supported use ENU or NED";
    }
}  

Odometry Glider::interpolate(int64_t timestamp)
{
    try
    {
        Odometry odom = factor_manager_.predict(timestamp);
        return odom;
    }
    catch (const std::exception& e)
    {
        LOG(ERROR) << "[GLIDER] Interpolation Error: " << e.what();
        return Odometry::Uninitialized();
    }
}

State Glider::optimize()
{
    try
    {   
        State state = factor_manager_.runner();
        return state;
    }
    catch (const std::exception& e)
    {
        LOG(ERROR) << "[GLIDER] Optimization Error: " << e.what();
        return State::Uninitialized();
    }
}
}
