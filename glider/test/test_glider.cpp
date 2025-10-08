/*!
* Jason Hughes
* October 2025
*
* Quick unit tests for the main glider module
*/

#include <gtest/gtest.h>

#include "glider/core/glider.hpp"

TEST(GliderTestSuite, AddTo)
{
    Glider::Glider glider("../config/graph-params.yaml");

    Eigen::Vector3d accel(0.0, 0.0, 9.81);
    Eigen::Vector3d gyro(0.0, 0.0, 0.0);
    Eigen::Vector4d orient(1.0, 0.0, 0.0, 0.0);
    int64_t timestamp = 1;

    glider.addImu(timestamp, accel, gyro, orient);

    double lat = 39.941278;
    double lon = -75.199199;

    Eigen::Vector3d gps(lat, lon, 0.0);

    glider.addGps(timestamp, gps);
}
