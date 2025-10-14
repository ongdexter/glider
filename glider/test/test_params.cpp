/*
* Unit test for parameters
*
*/
#include <gtest/gtest.h>

#include "glider/utils/parameters.hpp"

TEST(ParamsTestSuite, Constants)
{
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");
    
    // gravity can be positive or negative depending on
    // imu frame
    ASSERT_EQ(std::abs(params.gravity), 9.81);
    // we want at least 100 measurements
    ASSERT_GE(params.bias_num_measurements, 100);
    // lag time should not be too long
    ASSERT_LE(params.lag_time, 60.0);
}


TEST(ParamsTestSuite, Covariances)
{
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");

    // covariances should be positive
    ASSERT_GE(params.accel_cov, 0.0);
    ASSERT_GE(params.gyro_cov, 0.0);
    ASSERT_GE(params.heading_cov, 0.0);
    ASSERT_GE(params.roll_pitch_cov, 0.0);
    ASSERT_GE(params.integration_cov, 0.0);
    ASSERT_GE(params.bias_cov, 0.0);
    ASSERT_GE(params.gps_noise, 0.0);
}

TEST(ParamsTestSuite, Frame)
{ 
    Glider::Parameters params = Glider::Parameters::Load("../config/glider-params.yaml");

    EXPECT_TRUE(params.frame == "ned" || params.frame == "enu");
}
