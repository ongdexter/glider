/*
* Test the factor manager
*
*/

#include <gtest/gtest.h>

#include "glider/core/factor_manager.hpp"

TEST(FactorManagerTestSuite, ImuInitialization)
{
    // initialized glider factor manager and params
    Glider::Parameters params = Glider::Parameters::Load("../config/graph-params.yaml");
    Glider::FactorManager manager(params);

    // provide measurements for initialization
    for (int i = 0; i < params.bias_num_measurements + 10; ++i)
    {
        Eigen::Vector3d accel(0.0, 0.0, params.gravity);
        Eigen::Vector3d gyro(0.0, 0.0, 0.0);
        Eigen::Vector4d orient(1.0, 0.0, 0.0, 0.0);
        int64_t timestamp = i;
        manager.addImuFactor(timestamp, accel, gyro, orient);
    }

    // assert its initialized
    ASSERT_TRUE(manager.isImuInitialized());
    
    // get bias and compute mean
    Eigen::MatrixXd bias_est = manager.getBiasEstimate();

    Eigen::Vector3d accel_bias(bias_est.colwise().mean().head(3));
    Eigen::Vector3d gyro_bias(bias_est.colwise().mean().tail(3));
    
    Eigen::Vector3d accel_gt(0.0, 0.0, 0.0);
    Eigen::Vector3d gyro_gt(0.0, 0.0, 0.0);

    // check bias calculation is correct
    ASSERT_EQ(accel_bias, accel_gt);
    ASSERT_EQ(gyro_bias, gyro_gt);
}

