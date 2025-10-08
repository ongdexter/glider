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

std::mutex Glider::FactorManager::mutex_;

FactorManager::FactorManager(const Parameters& params)
{
    // set initialization status
    imu_initialized_ = false;
    gps_initialized_ = false;
    sys_initialized_ = false;

    // setup parameters
    params_ = params;
    imu_params_ = defaultImuParams(params.gravity);
    isam_params_ = gtsam::ISAM2Params();
    isam_params_.setRelinearizeThreshold(0.1);
    isam_params_.relinearizeSkip = 1;

    // imu initialization
    init_counter_ = 0;
    bias_estimate_vec_ = Eigen::MatrixXd::Zero(params.bias_num_measurements, 6);
    gravity_vec_ = Eigen::Vector3d(0.0, 0.0, params.gravity);

    LOG(INFO) << "[GLIDER] Factor Manager initialzed";
}

boost::shared_ptr<gtsam::PreintegrationCombinedParams> FactorManager::defaultImuParams(double g)
{
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> params;
    if (params_.frame == "enu")
    {
        params = gtsam::PreintegrationCombinedParams::MakeSharedD(g);
    }
    else
    {
        params = gtsam::PreintegrationCombinedParams::MakeSharedU(g);
    }
    double gyro_sigma = (0.5 * M_PI / 180.0) / 60.0;
    double accel_sigma = 0.001;
    
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    params->setGyroscopeCovariance(std::pow(gyro_sigma, 2) * I);
    params->setAccelerometerCovariance(std::pow(accel_sigma, 2) * I);
    params->setIntegrationCovariance(std::pow(0.0000001, 2) * I);

    return params;
}

void FactorManager::initializeImu(const Eigen::Vector3d& accel_meas, const Eigen::Vector3d& gyro_meas, const Eigen::Vector4d& orient) 
{
    if (init_counter_ < params_.bias_num_measurements)
    {
        // the measurement to bias matrix
        bias_estimate_vec_.row(init_counter_).head(3) = accel_meas - gravity_vec_;
        bias_estimate_vec_.row(init_counter_).tail(3) = gyro_meas;
        init_counter_++;
    }
    else
    {
        Eigen::VectorXd bias_mean = bias_estimate_vec_.colwise().mean();
        bias_ = gtsam::imuBias::ConstantBias(Eigen::Vector3d(bias_mean.head(3)),
                                             Eigen::Vector3d(bias_mean.tail(3)));

        std::cout << "Accel bias: " << Eigen::Vector3d(bias_mean.head(3)) << std::endl;
        std::cout << "Gyro Bias: " << Eigen::Vector3d(bias_mean.tail(3)) << std::endl;
        pim_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params_, bias_);
        imu_initialized_ = true;
        LOG(INFO) << "[GLIDER] IMU initalized";
    }
}

void FactorManager::addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps) 
{
    if (!imu_initialized_)
    {
        last_gps_ = gps;
        last_gps_time_ = nanosecIntToDouble(timestamp);
        return;
    }
}

void FactorManager::addImuFactor(int64_t timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient) 
{
    // if the imu is not initialized, pass the meaurements to initialize it
    if (!imu_initialized_)
    {
        last_imu_time_ = nanosecIntToDouble(timestamp);
        initializeImu(accel, gyro, orient);
        return;
    }
    // if the imu IS initialzied we want to add measurements to the pim
    double current_time = nanosecIntToDouble(timestamp);
    double dt = current_time - last_imu_time_;
    if (dt <= 0.0)
    {   
        LOG(WARNING) << "[GLIDER] Recieved IMU measurement out of order, ignoring";
        return;
    }
    // both the runner and the add imu access the pim in different threads
    // so we need to lock it when we manipulate it
    std::lock_guard<std::mutex> lock(mutex_);
    pim_->integrateMeasurement(accel, gyro, dt);
    last_imu_time_ = current_time;
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

bool FactorManager::isSystemInitialized() const
{
    return sys_initialized_;
}

bool FactorManager::isImuInitialized() const
{
    return imu_initialized_;
}

bool FactorManager::isGpsInitialized() const
{
    return gps_initialized_;
}

Eigen::MatrixXd FactorManager::getBiasEstimate() const
{
    return bias_estimate_vec_;
}
