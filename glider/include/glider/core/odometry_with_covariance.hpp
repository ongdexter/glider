/*
* Jason Hughes
* May 2025
*
* Object to keep track of the robots odometry with covariance. 
* This inherits from Odometry and adds functions to handle 
* covariance. This is used to track the results from 
* gtsam optimization.
*/

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/ImuBias.h>

#include <Eigen/Dense>
#include <utility>

#include "odometry.hpp"

using gtsam::symbol_shorthand::B; // Bias
using gtsam::symbol_shorthand::V; // Velocity
using gtsam::symbol_shorthand::X; // Pose

namespace Glider
{
class OdometryWithCovariance : public Odometry
{
    public:
        /*! @brief default constructor, make initialized_ flag false */
        OdometryWithCovariance() = default;
        /*! @brief main constructor used for results from gtsam optimization and covariance from optimizer
         *  @param val: results from the gtsam optimization
         *  @param key: the current key index of the factor graph
         *  @param pose_cov: a 9-by-9 covaraince matrix of the estimated pose from isam
         *  @param velocity_cov: a 3-by-3 covariance matrix of the estimated velocity from isam
         *  @param initialized: should the constructor call be an initialized odometry */
        OdometryWithCovariance(gtsam::Values& val, int64_t timestamp, gtsam::Key key, gtsam::Matrix& pose_cov, gtsam::Matrix& velocity_cov, bool initialized = true);
        // TODO I think this can be deprecated
        //OdometryWithCovariance(gtsam::Values& val, bool initialized = true);
        
        /*! @brief static method to initialize an Odometry that is not initialized
         *  same as calling the default constructor but better for readability */
        static OdometryWithCovariance Uninitialized();

        /*! @brief gets both the 3D accelerometer and 3D gyroscope biases
         *  @return the accelerometer and gyroscope biases as a gtsam or Eigen object
         *  @type T: gtsam::imuBias::ConstantBias
         *  @type T: std::pair<Eigen::Vector3d, Eigen::Vector3d> */
        template<typename T>
        T getBias() const;
        /*! @brief gets the accelerometer bias
         *  @return the 3D accelerometer bias vector
         *  @type T: gtsam::Vector3
         *  @type T: Eigen::Vector3d */
        template<typename T>
        T getAccelerometerBias() const; 
        /*! @brief gets the gyroscope bias
         *  @return the 3D gyroscope bias vector
         *  @type T: gtsam::Vector3
         *  @type T: Eigen::Vector3d */
        template<typename T>
        T getGyroscopeBias() const;
        /*! @brief gets the key index that was used during optimization
         *  to get these odometry values
         *  @return the key index
         *  @type T: gtsam::Key
         *  @type T: int */
        template<typename T>
        T getKeyIndex() const;
        
        /*! @brief gets the pose covaraince matrix
         *  @return the 9-by-9 pose covariance matrix */
        Eigen::MatrixXd getPoseCovariance() const;
        /*! @brief gets the position covariance matrix
         *  @return the 3-by-3 position covaraince matrix */
        Eigen::MatrixXd getPositionCovariance() const;
        /*! @brief gets the velocity covariance matrix
         *  @return the 3-by-3 velocity covariance matrix */
        Eigen::MatrixXd getVelocityCovariance() const;
        
        /*! @brief gets the key for the given symbol
         *  @param symbol: the symbol to use to get the key, should be 'X', 'V' or 'B'
         *  @return the key index and symbol in a formated string, ex: 'X0' */
        std::string getKeyIndex(const char* symbol);

        /*! @brief uses the velocity estimate to estimate if the
         *  the robot is moving
         *  @return true if the robot velocity is greater
         *  than some threshold otherwise false */
        bool isMoving() const;

        bool isMovingFasterThan(const double vel) const;

    private:
        // @brief is the robot moving or not
        bool is_moving_;
        
        // @brief stores the accelerometer bias
        gtsam::Vector3 accelerometer_bias_;
        // @brief stores the gyroscope bias
        gtsam::Vector3 gyroscope_bias_;

        // @brief the 9-by-9 pose covariance matrix
        gtsam::Matrix pose_covariance_;
        // @brief the 3-by-3 position covariance matrix
        gtsam::Matrix position_covariance_;
        // @brief the 3-by-3 velocity covariance matrix
        gtsam::Matrix velocity_covariance_;

        // @brief the key index that was used at optimization time
        gtsam::Key key_index_;
};
} // namespace glider
