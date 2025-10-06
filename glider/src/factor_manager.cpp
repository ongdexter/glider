/* Jason Hughes
 * May 2025
 *
 */
#include <mutex>

#include "glider/core/factor_manager.hpp"
#include "glider/utils/geodetics.hpp"
#include "glider/utils/gps_heading.hpp"

#include <gtsam/slam/expressions.h>

using namespace Glider;


FactorManager::FactorManager(const Parameters& params)
{
    // TODO
}

std::shared_ptr<gtsam::PreintegrationCombinedParams> FactorManager::defaultImuParams(double g)
{
    auto params = gtsam::PreintegrationCombinedParams::MakeSharedD(g);
    double gyro_sigma = (0.5 * M_PI / 180.0) / 60.0;
    double accel_sigma = 0.001;
    
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    params->setGyroscopeCovariance(std::pow(gyro_sigma, 2) * I);
    params->setAccelerometerCovariance(std::pow(accel_sigma, 2) * I);
    params->setIntegrationCovariance(std::pow(0.0000001, 2) * I);

    return params;
}

void FactorManager::imuInitialize(const Eigen::Vector3d& accel_meas, const Eigen::Vector3d& gyro_meas, const Eigen::Vector4d& orient) 
{
    // TODO
}

void FactorManager::addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps) 
{
    // TODO
}

void FactorManager::addImuFactor(int64_t timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient) 
{
    // TODO
}

Odometry FactorManager::predict(int64_t timestamp)
{
    // TODO
}

void FactorManager::initializeGraph() 
{
    // TODO
}

gtsam::Values FactorManager::optimize() 
{
    // TODO
}

State FactorManager::runner() 
{
    // TODO
}

gtsam::ExpressionFactorGraph FactorManager::getGraph()
{
    return graph_;
}

bool FactorManager::isInitialized()
{
    return initialized_;
}
