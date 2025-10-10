/*
* April 2025
* Manage the Factor Graph
*/

#pragma once

#include <glog/logging.h>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam_unstable/nonlinear/FixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/slam/InitializePose3.h>

#include "odometry_with_covariance.hpp"
#include "odometry.hpp"
#include "glider/utils/parameters.hpp"
#include "glider/utils/time.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>
#include <mutex>
#include <cmath>
#include <numeric>
#include <tuple>

// Symbol shorthand
using gtsam::symbol_shorthand::B; // Bias
using gtsam::symbol_shorthand::V; // Velocity
using gtsam::symbol_shorthand::X; // Pose

namespace Glider 
{

class FactorManager
{
    public:
        // Constructos
        FactorManager() = default;
        FactorManager(const Parameters& params);    
        
        // state predictors
        Odometry predict(int64_t timestamp); 
        OdometryWithCovariance runner(int64_t timestamp);

        // measurements adders
        void addGpsFactor(int64_t timestamp, const Eigen::Vector3d& gps);
        void addImuFactor(int64_t timestamp, const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro, const Eigen::Vector4d& orient);

        // getters and checkers 
        gtsam::ExpressionFactorGraph getGraph();
        bool isImuInitialized() const;
        bool isGpsInitialized() const;
        bool isSystemInitialized() const;
        Eigen::MatrixXd getBiasEstimate() const;
        gtsam::PreintegratedCombinedMeasurements getPim() const;
        gtsam::Key getKeyIndex() const;

    private: 
        gtsam::Values optimize();
        
        boost::shared_ptr<gtsam::PreintegrationCombinedParams> defaultImuParams(double g);
        
        void initializeGraph();
        void initializeImu(const Eigen::Vector3d& accel_meas, const Eigen::Vector3d& gyro_meas, const Eigen::Vector4d& orient);

        static std::mutex mutex_;

        // parameters
        std::map<std::string, Eigen::MatrixXd> matrix_config_;
        gtsam::ISAM2Params isam_params_;
        boost::shared_ptr<gtsam::PreintegrationCombinedParams> imu_params_;
        Parameters params_;

        // IMU
        int init_counter_;
        Eigen::Vector4d orient_;
        Eigen::Vector3d gravity_vec_;
        Eigen::MatrixXd bias_estimate_vec_;
        
        gtsam::imuBias::ConstantBias bias_;
        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> pim_;

        // noise
        gtsam::noiseModel::Isotropic::shared_ptr prior_noise_;
        gtsam::noiseModel::Isotropic::shared_ptr gps_noise_;
        gtsam::noiseModel::Base::shared_ptr orient_noise_;

        // factor graph
        uint64_t optimized_count_;
        gtsam::ExpressionFactorGraph graph_;
        gtsam::Values initials_;
        gtsam::Key key_index_;
        gtsam::IncrementalFixedLagSmoother smoother_;
        gtsam::FixedLagSmoother::KeyTimestampMap smoother_timestamps_;
        gtsam::ISAM2 isam_;

        // previous state
        Eigen::Vector3d last_gps_;
        gtsam::Matrix last_marginal_covariance_;
        double last_imu_time_;
        double last_gps_time_;

        // track states
        OdometryWithCovariance current_state_;
        OdometryWithCovariance last_state_;

        // initial state
        gtsam::Rot3 initial_orientation_;

        // initialization
        bool sys_initialized_;
        bool imu_initialized_;
        bool gps_initialized_;
};
}
