/*
* Jason Hughes
* July 2025
*
* takes care of translation of all inputs into the correct frame
* which is the ENU frame.
*/

#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <glog/logging.h>

#include "glider/core/factor_manager.hpp"
#include "glider/core/differential_gps.hpp"
#include "glider/utils/geodetics.hpp"

namespace Glider
{

class Glider
{
    public:
        /*! @brief the default constructor for glider */
        Glider() = default;
        /*! @brief constuctor that initilizes necessary parameters
         *  @param path: path to the params yaml file */
        Glider(const std::string& path);

        /*! @brief converts the gps measurement from lat, lon to UTM
         *  and passes that to the factor manager
         *  @param timestamp: time that the gps measurement was taken
         *  @param gps: gps measurement in (lat, lon, alt) format, 
         *  should be in degree decimal and altitude in meters. Altitude
         *  frame does not matter */
        void addGps(int64_t timestamp, Eigen::Vector3d& gps);
        void addGpsWithHeading(int64_t timestamp, Eigen::Vector3d& gps);
        /*! @brief converts the imu measurements into the ENU frame if
         *  they are not in that frame already.
         *  @param timestamp: time the imu measurement was taken
         *  @param accel: acceleration measurement in the imu's frame
         *  @param gyro: gyroscope measurement in the imu's frame
         *  @param quat: the orientation measurement in the imu's frame */
        void addImu(int64_t timestamp, Eigen::Vector3d& accel, Eigen::Vector3d& gyro, Eigen::Vector4d& quat);

        /*! @brief calls the factor manager to interpolate between GPS 
         *  measurements using the pim
         *  @param timestamp: time at which you want to interpolate 
         *  @return odometry object tracking the predicted navstate
         *  from the pim */
        Odometry interpolate(int64_t timestamp);
        /*! @brief calls the factor manager to optimize over the factor graph
         *  @param timestamp: time at which you want to optimize
         *  @return the full odometry estimate with covariance from the results
         *  of the gtsam optimization */
        OdometryWithCovariance optimize(int64_t timestamp);
        
    private:
        /*! @brief initializes glog with the specified logging
         *  parameters
         *  @params: the laoded params from the yaml file */
        void initializeLogging(const Parameters& params) const;
        /*! @brief rotate a quaternion by a rotation matrix
         *  @params rot: the rotation matrix you want to rotate the quaterniuon by
         *  @param quat: the orientation we want to rotate
         *  @return the rotated orientation as quaternion as a Vector4d in
         *  (w, x, y, z) format */
        Eigen::Vector4d rotateQuaternion(const Eigen::Matrix3d& rot, const Eigen::Vector4d& quat) const;

        // @brief the factor manager, handles the factor graph
        FactorManager factor_manager_;

        // @brief the frame of the imu device use, should be
        // "ned" or "enu", as specified in the params file
        std::string frame_;
        // @brief the relative translation from the gps to
        // the imu
        Eigen::Vector3d t_imu_gps_;
        // @brief the rotation matrix from ned to enu frame
        Eigen::Matrix3d r_enu_ned_;
        // @brief whether or not to use differential gps
        // from motion for heading
        bool use_dgpsfm_;
        // @brief object to handle differential gps 
        // from motion
        Geodetics::DifferentialGpsFromMotion dgps_;
        // @brief save the state estimate from 
        // the optimizer
        OdometryWithCovariance current_odom_;
};
}
