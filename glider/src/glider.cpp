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
    r_enu_ned_ = Eigen::Matrix3d::Zero();
    r_enu_ned_ << 0.0, 1.0, 0.0,
                  1.0, 0.0, 0.0,
                  0.0, 0.0, -1.0;

    LOG(INFO) << "[GLIDER] Using IMU frame: " << frame_;
    LOG(INFO) << "[GLIDER] Using Fixed Lag Smoother: " << std::boolalpha << params.smooth;
    LOG(INFO) << "[GLIDER] Logging to: " << params.log_dir;    
    use_dgpsfm_ = params.use_dgpsfm;
    dgps_ = Geodetics::DifferentialGpsFromMotion(params.frame, params.dgpsfm_threshold);

    current_odom_ = OdometryWithCovariance::Uninitialized();

    LOG(INFO) << "[GLIDER] Using IMU frame: " << frame_;
    LOG(INFO) << "[GLIDER] Using Fixed Lag Smoother: " << std::boolalpha << params.smooth;
    LOG(INFO) << "[GLIDER] Using DGPS From Motion: " << std::boolalpha << params.use_dgpsfm;
    LOG(INFO) << "[GLIDER] Glider initialized";
}

void Glider::initializeLogging(const Parameters& params) const
{
    // initialize GLog
    google::InitGoogleLogging("Glider");

    FLAGS_log_dir = params.log_dir;
    if (params.log) FLAGS_alsologtostderr = 1;
}


void Glider::addGps(int64_t timestamp, Eigen::Vector3d& gps)
{
    // route the
    if (use_dgpsfm_)
    {
        addGpsWithHeading(timestamp, gps);
        return;
    }

    // transform from lat lon To UTM
    Eigen::Vector3d meas = Eigen::Vector3d::Zero();
    
    double easting, northing;
    char zone[4];
    geodetics::LLtoUTM(gps(0), gps(1), northing, easting, zone);
    
    // keep everything in the enu frame
    meas.head(2) << easting, northing;
    meas(2) = gps(2);

    // TODO t_imu_gps_ needs to be rotated!!
    meas = meas + t_imu_gps_;

    factor_manager_.addGpsFactor(timestamp, meas);
}

void Glider::addGpsWithHeading(int64_t timestamp, Eigen::Vector3d& gps)
{
    // transform from lat lon To UTM
    Eigen::Vector3d meas = Eigen::Vector3d::Zero();
    
    double easting, northing;
    char zone[4];
    geodetics::LLtoUTM(gps(0), gps(1), northing, easting, zone);
    
    // keep everything in the enu frame
    meas.head(2) << easting, northing;
    meas(2) = gps(2);

    // TODO t_imu_gps_ needs to be rotated!!
    meas = meas + t_imu_gps_;
    
    if(factor_manager_.isSystemInitialized() && current_odom_.isMovingFasterThan(dgps_.getVelocityThreshold()))
    {
        double heading = dgps_.getHeading(gps);
        factor_manager_.addGpsFactor(timestamp, meas, heading, true);
    }
    else
    {
        dgps_.setLastGps(gps);
        factor_manager_.addGpsFactor(timestamp, meas, 0.0, false);
    }
}

void Glider::addImu(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d& gyro, Eigen::Vector4d& quat)
{
    if (frame_ == "ned")
    {
        Eigen::Vector3d accel_enu = r_enu_ned_ * accel;
        Eigen::Vector3d gyro_enu = r_enu_ned_ * gyro;
        Eigen::Vector4d quat_enu = rotateQuaternion(r_enu_ned_, quat);

        factor_manager_.addImuFactor(timestamp, accel_enu, gyro_enu, quat_enu);
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

OdometryWithCovariance Glider::optimize(int64_t timestamp)
{
    try
    {   
        current_odom_ = factor_manager_.runner(timestamp);
        return current_odom_;
    }
    catch (const std::exception& e)
    {
        LOG(ERROR) << "[GLIDER] Optimization Error: " << e.what();
        return OdometryWithCovariance::Uninitialized();
    }
}

Eigen::Vector4d Glider::rotateQuaternion(const Eigen::Matrix3d& rot, const Eigen::Vector4d& quat) const
{
    Eigen::Quaterniond q_ned(quat(0), quat(1), quat(2), quat(3));
    Eigen::Quaterniond q_ned_enu(rot);

    Eigen::Quaterniond q_enu = q_ned_enu * q_ned;

    return Eigen::Vector4d(q_enu.w(), q_enu.x(), q_enu.y(), q_enu.z());
}
}
