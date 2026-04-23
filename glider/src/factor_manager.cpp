/* Jason Hughes
 * May 2025
 *
 */
#include <mutex>

#include "glider/core/factor_manager.hpp"
#include "glider/utils/geodetics.hpp"

#include <gtsam/slam/expressions.h>

using namespace Glider;

std::mutex Glider::FactorManager::mutex_;

FactorManager::FactorManager(const Parameters& params)
{
    initialize(params);
}

void FactorManager::initialize(const Parameters& params)
{
    // set initialization status
    imu_initialized_ = false;
    gps_initialized_ = false;
    sys_initialized_ = false;
    odom_initialized_ = false;
    using_local_origin_ = false;
    gps_offset_initialized_ = false;
    gps_offset_ = Eigen::Vector3d::Zero();
    last_node_time_ = 0.0;
    accumulated_odom_delta_ = Eigen::Isometry3d::Identity();

    // setup parameters
    params_ = params;
    imu_params_ = defaultImuParams(params.gravity);


    // imu initialization
    init_counter_ = 0;
    bias_estimate_vec_ = Eigen::MatrixXd::Zero(params.bias_num_measurements, 6);
    gravity_vec_ = Eigen::Vector3d(0.0, 0.0, params.gravity);

    // set noise model
    gps_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, params.gps_noise);
    orient_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(params.roll_pitch_cov, params.roll_pitch_cov, params.heading_cov));
    dgpsfm_noise_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(M_PI/2, M_PI/2, params.dgpsfm_cov));
    odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << params.odom_cov, params.odom_cov, params.odom_cov, params.odom_cov, params.odom_cov, params.odom_cov).finished());
    
    // set key index
    key_index_ = 0;

    // setup graph
    optimized_count_ = 0;
    isam_params_ = gtsam::ISAM2Params();
    isam_params_.setRelinearizeThreshold(0.1);
    isam_params_.relinearizeSkip = 1;
    isam_ = gtsam::ISAM2(isam_params_);
    smoother_ = gtsam::IncrementalFixedLagSmoother(params_.lag_time, isam_params_);
    
    orient_ = Eigen::Vector4d(1.0, 0.0, 0.0, 0.0);

    LOG(INFO) << "[GLIDER] Factor Manager initialzed";
}

boost::shared_ptr<gtsam::PreintegrationCombinedParams> FactorManager::defaultImuParams(double g)
{
    boost::shared_ptr<gtsam::PreintegrationCombinedParams> params;
    params = gtsam::PreintegrationCombinedParams::MakeSharedU(g);
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

        // initialize the pim
        pim_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params_, bias_);
        // initialize the orientation
        initial_orientation_ = gtsam::Rot3::Quaternion(orient(0), orient(1), orient(2), orient(3));
        // initialize the graph once the imu is initialized
        initializeGraph();

        imu_initialized_ = true;
        LOG(INFO) << "[GLIDER] IMU initialized (bias calibration complete)";
    }
}

void FactorManager::initializeGraph() 
{
    initials_ = gtsam::InitializePose3::initialize(graph_);
}

void FactorManager::addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps, const double sigma) 
{
    std::lock_guard<std::mutex> graph_lock(graph_mutex_);
    double time = nanosecIntToDouble(timestamp);

    // wait until the imu is initialized
    if (!imu_initialized_) 
    {
        LOG_FIRST_N(WARNING, 5) << "[GLIDER] GPS received but IMU is not initialized yet. Skipping factor.";
        return;
    }

    if (!gps_offset_initialized_)
    {
        if (using_local_origin_)
        {
            gps_offset_ = gps - current_state_.getPose<gtsam::Pose3>().translation();
            gps_offset_initialized_ = true;
            gps_initialized_ = true;
            LOG(INFO) << "[GLIDER] GPS offset initialized from outdoor transition at " << std::fixed << std::setprecision(2) << gps.transpose();
        }
        else
        {
            gps_offset_ = Eigen::Vector3d::Zero();
            gps_offset_initialized_ = true;
            gps_initialized_ = true;
            LOG(INFO) << "[GLIDER] GPS offset initialized (GPS is origin)";
        }
    }
    
    if (key_index_ == 0)
    {
        // set the initial NavState 
        // The initial orientation is the the initial orientation from the imu initialization
        // The initial position is from the gps 
        // Initial velocity is set to zero
        gtsam::Pose3 initial_pose = gtsam::Pose3(initial_orientation_, gtsam::Point3(gps(0), gps(1), gps(2)));
        gtsam::NavState initial_navstate(initial_pose, gtsam::Point3(0.0, 0.0, 0.0)); // TODO why do I need this??
        
        // save the initial values
        initials_.insert(X(key_index_), initial_pose);
        initials_.insert(V(key_index_), gtsam::Point3(0.0, 0.0, 0.0));
        initials_.insert(B(key_index_), bias_);

        // save the timestamps for the smoother
        smoother_timestamps_[X(key_index_)] = time;
        smoother_timestamps_[V(key_index_)] = time;
        smoother_timestamps_[B(key_index_)] = time;

        // add prior factors to the graph
        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_index_), initial_pose, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::Point3>(V(key_index_), gtsam::Point3(0.0, 0.0, 0.0), gtsam::noiseModel::Isotropic::Sigma(3, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(key_index_), bias_, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));

        key_index_++;
        gps_initialized_ = true;
        LOG(INFO) << "[GLIDER] GPS Initialized";
        return;
    }
   
    // add the pim to the graph under a mutex
    std::unique_lock<std::mutex> pim_lock(mutex_);
    graph_.add(gtsam::CombinedImuFactor(X(key_index_-1), V(key_index_-1), X(key_index_), V(key_index_), B(key_index_-1), B(key_index_), *pim_));
    pim_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params_, bias_);
    if (odom_initialized_)
    {
        gtsam::Pose3 odom_delta_gtsam(accumulated_odom_delta_.matrix());
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(key_index_-1), X(key_index_), odom_delta_gtsam, odom_noise_));
    }
    accumulated_odom_delta_ = Eigen::Isometry3d::Identity();
    pim_lock.unlock();

    // insert new initial values
    if (!initials_.exists(X(key_index_))) initials_.insert(X(key_index_), current_state_.getPose<gtsam::Pose3>());
    if (!initials_.exists(V(key_index_))) initials_.insert(V(key_index_), current_state_.getVelocity<gtsam::Vector3>());
    if (!initials_.exists(B(key_index_))) initials_.insert(B(key_index_), bias_);

    // save the time for the smoother
    smoother_timestamps_[X(key_index_)] = time;
    smoother_timestamps_[V(key_index_)] = time;
    smoother_timestamps_[B(key_index_)] = time;
    
    // convert eigen to gtsam
    gtsam::Point3 meas(gps(0), gps(1), gps(2));
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(orient_(0), orient_(1), orient_(2), orient_(3));


    Eigen::Vector3d aligned_gps = gps;
    if (using_local_origin_) aligned_gps = gps - gps_offset_;

    gtsam::noiseModel::Isotropic::shared_ptr noise = gps_noise_;
    if (sigma > 0.0) noise = gtsam::noiseModel::Isotropic::Sigma(3, sigma);

    // add gps measurement to factor graph as gtsam object
    graph_.add(gtsam::GPSFactor(X(key_index_), aligned_gps, noise));
    graph_.addExpressionFactor(gtsam::rotation(X(key_index_)), rot, orient_noise_);
 
    // increment key index
    key_index_++;
    last_node_time_ = time;
}

void FactorManager::addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps, const double& heading, const bool fuse, const double sigma) 
{
    std::lock_guard<std::mutex> graph_lock(graph_mutex_);
    double time = nanosecIntToDouble(timestamp);

    // wait until the imu is initialized
    if (!imu_initialized_) 
    {
        LOG_FIRST_N(WARNING, 5) << "[GLIDER] GPS received but IMU is not initialized yet. Skipping factor.";
        return;
    }

    if (!gps_offset_initialized_)
    {
        if (using_local_origin_)
        {
            gps_offset_ = gps - current_state_.getPose<gtsam::Pose3>().translation();
            gps_offset_initialized_ = true;
            gps_initialized_ = true;
            LOG(INFO) << "[GLIDER] GPS offset initialized from outdoor transition at " << std::fixed << std::setprecision(2) << gps.transpose();
        }
        else
        {
            gps_offset_ = Eigen::Vector3d::Zero();
            gps_offset_initialized_ = true;
            gps_initialized_ = true;
            LOG(INFO) << "[GLIDER] GPS offset initialized (GPS is origin)";
        }
    }
    
    if (key_index_ == 0)
    {
        // set the initial NavState 
        // The initial orientation is the the initial orientation from the imu initialization
        // The initial position is from the gps 
        // Initial velocity is set to zero
        gtsam::Pose3 initial_pose = gtsam::Pose3(initial_orientation_, gtsam::Point3(gps(0), gps(1), gps(2)));
        gtsam::NavState initial_navstate(initial_pose, gtsam::Point3(0.0, 0.0, 0.0)); // TODO why do I need this??
        
        // save the initial values
        initials_.insert(X(key_index_), initial_pose);
        initials_.insert(V(key_index_), gtsam::Point3(0.0, 0.0, 0.0));
        initials_.insert(B(key_index_), bias_);

        // save the timestamps for the smoother
        smoother_timestamps_[X(key_index_)] = time;
        smoother_timestamps_[V(key_index_)] = time;
        smoother_timestamps_[B(key_index_)] = time;

        // add prior factors to the graph
        graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_index_), initial_pose, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::Point3>(V(key_index_), gtsam::Point3(0.0, 0.0, 0.0), gtsam::noiseModel::Isotropic::Sigma(3, 0.001)));
        graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(key_index_), bias_, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));

        key_index_++;
        gps_initialized_ = true;
        LOG(INFO) << "[GLIDER] GPS Initialized";
        return;
    }
   
    // add the pim to the graph under a mutex
    std::unique_lock<std::mutex> pim_lock(mutex_);
    graph_.add(gtsam::CombinedImuFactor(X(key_index_-1), V(key_index_-1), X(key_index_), V(key_index_), B(key_index_-1), B(key_index_), *pim_));
    pim_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params_, bias_);
    if (odom_initialized_)
    {
        gtsam::Pose3 odom_delta_gtsam(accumulated_odom_delta_.matrix());
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(key_index_-1), X(key_index_), odom_delta_gtsam, odom_noise_));
    }
    accumulated_odom_delta_ = Eigen::Isometry3d::Identity();
    pim_lock.unlock();

    // insert new initial values
    if (!initials_.exists(X(key_index_))) initials_.insert(X(key_index_), current_state_.getPose<gtsam::Pose3>());
    if (!initials_.exists(V(key_index_))) initials_.insert(V(key_index_), current_state_.getVelocity<gtsam::Vector3>());
    if (!initials_.exists(B(key_index_))) initials_.insert(B(key_index_), bias_);

    // save the time for the smoother
    smoother_timestamps_[X(key_index_)] = time;
    smoother_timestamps_[V(key_index_)] = time;
    smoother_timestamps_[B(key_index_)] = time;
    
    // add gps measurement to factor graph as gtsam object
    gtsam::Point3 meas(gps(0), gps(1), gps(2));
    double heading_rad = heading * M_PI / 180.0;
    gtsam::Rot3 rot = gtsam::Rot3::Ypr(heading_rad, 0.0, 0.0);


    Eigen::Vector3d aligned_gps = gps;
    if (using_local_origin_) aligned_gps = gps - gps_offset_;

    gtsam::noiseModel::Isotropic::shared_ptr noise = gps_noise_;
    if (sigma > 0.0) noise = gtsam::noiseModel::Isotropic::Sigma(3, sigma);

    // add gps measurement to factor graph as gtsam object
    graph_.add(gtsam::GPSFactor(X(key_index_), aligned_gps, noise));
    if (fuse) graph_.addExpressionFactor(gtsam::rotation(X(key_index_)), rot, dgpsfm_noise_);

    // increment key index
    key_index_++;
    last_node_time_ = time;
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
    // if the imu is initialzied we want to add measurements to the pim
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
    orient_ = orient;

    last_imu_time_ = current_time;
}

bool FactorManager::addOdomFactor(int64_t timestamp, const Eigen::Isometry3d& odom) 
{
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (!imu_initialized_) return false;

    std::lock_guard<std::mutex> pim_lock(mutex_);
    if (!odom_initialized_)
    {
        last_odom_meas_ = odom;
        odom_initialized_ = true;
        
        if (key_index_ == 0)
        {
            double time = nanosecIntToDouble(timestamp);
            gtsam::Pose3 initial_pose(odom.matrix());
            if (!initials_.exists(X(key_index_))) initials_.insert(X(key_index_), initial_pose);
            if (!initials_.exists(V(key_index_))) initials_.insert(V(key_index_), gtsam::Point3(0.0, 0.0, 0.0));
            if (!initials_.exists(B(key_index_))) initials_.insert(B(key_index_), bias_);

            smoother_timestamps_[X(key_index_)] = time;
            smoother_timestamps_[V(key_index_)] = time;
            smoother_timestamps_[B(key_index_)] = time;

            graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(key_index_), initial_pose, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));
            graph_.add(gtsam::PriorFactor<gtsam::Point3>(V(key_index_), gtsam::Point3(0.0, 0.0, 0.0), gtsam::noiseModel::Isotropic::Sigma(3, 0.001)));
            graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(key_index_), bias_, gtsam::noiseModel::Isotropic::Sigma(6, 0.001)));

            key_index_++;
            using_local_origin_ = true;
            last_node_time_ = time;
            LOG(INFO) << "[GLIDER] Odometry Initialized Graph Origin";
            return true;
        }

        LOG(INFO) << "[GLIDER] Odometry Tracking Begun";
        return false;
    }

    Eigen::Isometry3d delta = last_odom_meas_.inverse() * odom;
    accumulated_odom_delta_ = accumulated_odom_delta_ * delta;
    last_odom_meas_ = odom;

    // only create a new node if we moved > 0.3 meters or rotated > 0.15 rad or if we are trying to initialize the system
    double trans_dist = accumulated_odom_delta_.translation().norm();
    Eigen::AngleAxisd angle_axis(accumulated_odom_delta_.rotation());
    double rot_dist = std::abs(angle_axis.angle());

    double time = nanosecIntToDouble(timestamp);
    double dt = time - last_node_time_;

    if (trans_dist > 0.3 || rot_dist > 0.15 || (!sys_initialized_ && key_index_ < params_.initial_num_measurements + 2) || dt > 1.0)
    {
        graph_.add(gtsam::CombinedImuFactor(X(key_index_-1), V(key_index_-1), X(key_index_), V(key_index_), B(key_index_-1), B(key_index_), *pim_));
        pim_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imu_params_, bias_);

        gtsam::Pose3 odom_delta_gtsam(accumulated_odom_delta_.matrix());
        graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(X(key_index_-1), X(key_index_), odom_delta_gtsam, odom_noise_));

        accumulated_odom_delta_ = Eigen::Isometry3d::Identity();

        gtsam::Pose3 next_pose = current_state_.isInitialized() ? current_state_.getPose<gtsam::Pose3>() : gtsam::Pose3(odom.matrix());
        gtsam::Vector3 next_vel = current_state_.isInitialized() ? current_state_.getVelocity<gtsam::Vector3>() : gtsam::Vector3(0.0, 0.0, 0.0);

        if (!initials_.exists(X(key_index_))) initials_.insert(X(key_index_), next_pose);
        if (!initials_.exists(V(key_index_))) initials_.insert(V(key_index_), next_vel);
        if (!initials_.exists(B(key_index_))) initials_.insert(B(key_index_), bias_);

        smoother_timestamps_[X(key_index_)] = time;
        smoother_timestamps_[V(key_index_)] = time;
        smoother_timestamps_[B(key_index_)] = time;

        key_index_++;
        last_node_time_ = time;
        return true;
    }
    
    return false;
}

void FactorManager::addLandmarkFactor(int64_t /*timestamp*/, size_t landmark_id, const Eigen::Vector3d& utm, const Eigen::Matrix3d& cov)
{
    std::lock_guard<std::mutex> lock(graph_mutex_);
    Eigen::Matrix3d obs_info = cov.inverse();
    auto it = landmark_info_.find(landmark_id);
    if (it == landmark_info_.end()) {
        landmark_info_[landmark_id] = obs_info;
        landmark_info_vec_[landmark_id] = obs_info * utm;
    } else {
        it->second += obs_info;
        landmark_info_vec_[landmark_id] += obs_info * utm;
    }
}

PointWithCovariance FactorManager::getLandmarkPoint(size_t landmark_id) const
{
    auto it = landmark_info_.find(landmark_id);
    if (it == landmark_info_.end()) return PointWithCovariance();
    
    Eigen::Matrix3d cov = it->second.inverse();
    Eigen::Vector3d point = cov * landmark_info_vec_.at(landmark_id);
    return PointWithCovariance(point, cov);
}

Odometry FactorManager::predict(int64_t timestamp)
{
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (isSystemInitialized() && pim_)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        gtsam::NavState result = pim_->predict(current_state_.getNavState(), bias_);
        Odometry odom(result, timestamp, true);
        odom.setGpsOffsetInitialized(gps_offset_initialized_);
        return odom;
    }
    
    return Odometry::Uninitialized();
}


gtsam::Values FactorManager::optimize() 
{
    gtsam::Values result;
    // call the specified optimizer
    if (params_.smooth)
    {
        smoother_.update(graph_, initials_, smoother_timestamps_);
        result = smoother_.calculateEstimate();
    }
    else
    {
        isam_.update(graph_, initials_);
        result = isam_.calculateEstimate();
    }
    optimized_count_++;
    // if weve optimized the specified number of times, initialize the system
    if (optimized_count_ == params_.initial_num_measurements)
    {
        LOG(INFO) << "[GLIDER] System Initialized";
        sys_initialized_ = true;
    }

    return result;
}

OdometryWithCovariance FactorManager::runner(int64_t timestamp) 
{
    std::lock_guard<std::mutex> lock(graph_mutex_);
    // if the graph or imu is not initialized we cannot optimize
    // so we return an uninitialized state
    if (!isSystemInitialized() || !imu_initialized_) 
    {
        return OdometryWithCovariance::Uninitialized();
    }

    gtsam::Values result;
    try
    {
        result = optimize();
    }
    catch (const std::exception& e)
    {
        graph_.resize(0);
        initials_.clear();
        smoother_timestamps_.clear();
        // Roll back key_index_ to the last successfully optimized key
        // so the next GPS/odom measurement creates factors referencing
        // keys that actually exist in ISAM2/smoother
        if (current_state_.isInitialized())
        {
            key_index_ = current_state_.getKeyIndex<int>() + 1;
            LOG(WARNING) << "[GLIDER] Optimizer failed, rolling back key_index_ to " << key_index_;
        }
        else
        {
            key_index_ = 0;
            LOG(WARNING) << "[GLIDER] Optimizer failed during init, resetting key_index_ to 0";
        }
        throw;
    }

    // get the covariance from isam or the smoother
    gtsam::Matrix pose_cov, vel_cov;
    if (params_.smooth)
    {
        pose_cov = smoother_.marginalCovariance(X(key_index_-1));
        vel_cov = smoother_.marginalCovariance(V(key_index_-1));
    }
    else
    {
        pose_cov = isam_.marginalCovariance(X(key_index_-1));
        vel_cov = isam_.marginalCovariance(V(key_index_-1));
    }
    // save the current state we just optimized for
    current_state_ = OdometryWithCovariance(result, timestamp, key_index_-1, pose_cov, vel_cov, true);
    current_state_.setGpsOffsetInitialized(gps_offset_initialized_);

    bias_ = current_state_.getBias<gtsam::imuBias::ConstantBias>();

    // reset the graph
    graph_.resize(0);
    initials_.clear();
    smoother_timestamps_.clear();

    // we want to optimize a few times before
    // publishing to allow convergence
    // otherwise we return an unitialized state
    if (!isSystemInitialized()) return OdometryWithCovariance::Uninitialized();

    return current_state_;
}

gtsam::ExpressionFactorGraph FactorManager::getGraph()
{
    return graph_;
}

bool FactorManager::isSystemInitialized() const 
{ 
    return key_index_ > 0; 
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

gtsam::PreintegratedCombinedMeasurements FactorManager::getPim() const
{
    return *pim_;
}

gtsam::Key FactorManager::getKeyIndex() const
{
    return key_index_;
}

Eigen::Vector3d FactorManager::getGpsOffset() const
{
    return gps_offset_;
}

bool FactorManager::isGpsOffsetInitialized() const
{
    return gps_offset_initialized_;
}
