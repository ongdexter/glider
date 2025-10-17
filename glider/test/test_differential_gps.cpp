/*!
* Jason Hughes
* October 2025
*
* test the differential gps 
* object
*/

#include <gtest/gtest.h>

#include "glider/core/differential_gps.hpp"

static const double LAT = 32.925;
static const double LON = -75.199;
static const double ALT = 10.0;
static const double VEL = 1.0;

TEST(DifferentialGpsTestSuite, TestInitialization)
{
    Glider::Geodetics::DifferentialGpsFromMotion dgps("enu", VEL);
    Eigen::Vector3d gps(LAT, LON, ALT);

    dgps.setLastGps(gps);

    ASSERT_TRUE(dgps.isInitialized());
}

TEST(DifferentialGpsTestSuite, TestNorthHeading)
{
    Glider::Geodetics::DifferentialGpsFromMotion dgps("enu", VEL);

    double nlat = 39.942136;
    double nlon = -75.19969;
    double slat = 39.942041;
    double slon = -75.199694;

    dgps.setLastGps(Eigen::Vector3d(slat, slon, ALT));

    double heading = dgps.getHeading(Eigen::Vector3d(nlat, nlon, ALT));

    ASSERT_NEAR(heading, M_PI/2, 0.1);
}

TEST(DifferentialGpsTestSuite, TestSouthHeading)
{
    Glider::Geodetics::DifferentialGpsFromMotion dgps("enu", VEL);

    double nlat = 39.942136;
    double nlon = -75.19969;
    double slat = 39.942041;
    double slon = -75.199694;

    dgps.setLastGps(Eigen::Vector3d(nlat, nlon, ALT));
    
    double heading = dgps.getHeading(Eigen::Vector3d(slat, slon, ALT));

    ASSERT_NEAR(heading, 3*M_PI/2, 0.1);
}

TEST(DifferentialGpsTestSuite, TestEastHeading)
{
    Glider::Geodetics::DifferentialGpsFromMotion dgps("enu", VEL);

    double wlat = 39.942197; 
    double wlon = -75.199524;
    double elat = 39.942193;
    double elon = -75.199374;

    dgps.setLastGps(Eigen::Vector3d(wlat, wlon, ALT));

    double heading = dgps.getHeading(Eigen::Vector3d(elat, elon, ALT));

    ASSERT_NEAR(heading, 2*M_PI, 0.1);
}

TEST(DifferentialGpsTestSuite, TestWestHeading)
{
    Glider::Geodetics::DifferentialGpsFromMotion dgps("enu", VEL);

    double wlat = 39.942197; 
    double wlon = -75.199524;
    double elat = 39.942193;
    double elon = -75.199374;

    dgps.setLastGps(Eigen::Vector3d(elat, elon, ALT));

    double heading = dgps.getHeading(Eigen::Vector3d(wlat, wlon, ALT));

    ASSERT_NEAR(heading, M_PI, 0.1);
}

TEST(DifferentialGpsTestSuite, TestVelocityThreshold)
{
    Glider::Geodetics::DifferentialGpsFromMotion dgps("enu", VEL);

    ASSERT_EQ(VEL, dgps.getVelocityThreshold());

    ASSERT_TRUE(dgps.isIntegratable(Eigen::Vector3d(2.0, 0.0, 0.0)));
    ASSERT_FALSE(dgps.isIntegratable(Eigen::Vector3d(0.5, 0.0, 0.0)));
}
