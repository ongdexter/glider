/*
* Jason Hughes
* July 2025
*
* main pose graph code
*/

#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <glog/logging.h>

#include "glider/core/factor_manager.hpp"
#include "glider/utils/geodetics.hpp"

namespace Glider
{

class Glider
{
    public:
        Glider() = default;
        Glider(const std::string& path);

        void addGps(int64_t timestamp, Eigen::Vector3d& gps);
        void addImu(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d& gyro, Eigen::Vector4d& quat);

        Odometry interpolate(int64_t timestamp);
        OdometryWithCovariance optimize();
        
    private:

        void initializeLogging(const Parameters& params) const;

        FactorManager factor_manager_;

        std::string frame_;
        Eigen::Vector3d t_imu_gps_;
};
}
