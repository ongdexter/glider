/*
* Jason Hughes
* July 2025
*
* Take care of all necessary transforms.
*/

#include "glider/core/factor_manager.hpp"
#include "glider/core/glider.hpp"

namespace Glider
{

Glider::Glider(const std::string& path) 
{
    Parameters params = Parameters::Load(path);
    factor_manager_ = FactorManager(params);

    frame_ = params.frame;
    correct_imu_ = params.correct_imu;
    t_imu_gps_ = params.t_imu_gps;

    // IMU transformations
    ned_to_enu_rot_ << 0.0, 1.0, 0.0, 
                       1.0, 0.0, 0.0, 
                       0.0, 0.0, -1.0;
    ned_to_enu_quat_ = Eigen::Quaterniond(ned_to_enu_rot_);

    prev_pose_ = Eigen::Isometry3d::Identity();

    current_heading_ = 0.0;
    initial_heading_ = 0.0;
    set_initial_heading_ = true;

    std::cout << "[GLIDER] Using IMU frame: " << frame_ << std::endl;
    std::cout << "[GLIDER] Correcting IMU heading with mag: " << std::boolalpha << correct_imu_ << std::endl;
    std::cout << "[GLIDER] Glider initialized" << std::endl;
}

void Glider::addGPS(int64_t timestamp, Eigen::Vector3d& gps)
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

void Glider::addIMU(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d& gyro, Eigen::Vector4d& quat)
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
        throw std::runtime_error("IMU Frame, not supported use ENU or NED");
        return;
    }
}  

void Glider::addOdom(int64_t timestamp, Eigen::Isometry3d& pose)
{
//    if (factor_manager_.isInitialized() && !set_initial_heading_)
//    {
//        // Convert orb pose to ENU
//        Eigen::Matrix3d odom_to_enu;
//        double h = initial_heading_;
//        odom_to_enu << std::cos(h), -std::sin(h), 0.0, 
//                      std::sin(h), std::cos(h), 0.0, 
//                      0.0, 0.0, 1.0;
//        Eigen::Isometry3d enu_pose = pose.rotate(odom_to_enu);
//        
//        // calculate the relative pose in ENU frame
//        Eigen::Isometry3d rel_pose = prev_pose_.inverse() * enu_pose;
//        
//        // convert to gtsam for pose graph
//        gtsam::Pose3 gtpose = isometryToPose(rel_pose);
//        factor_manager_.addOdometryFactor(timestamp, gtpose);
//        prev_pose_ = enu_pose;
//    }
//    else
//    {
//        prev_pose_ = pose;
//    }
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
