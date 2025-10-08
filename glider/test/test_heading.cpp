/*
*
*
*/

#include <gtest/gtest.h>

#include "glider/utils/gps_heading.hpp"

TEST(GPSHeadingTestSuite, NorthHeading)
{
    double nlat = 39.942136;
    double nlon = -75.19969;
    double slat = 39.942041;
    double slon = -75.199694;
    double heading = Glider::geodetics::gpsHeading(slat, slon, nlat, nlon);

    EXPECT_NEAR(heading, 0.0, 0.1);
}

TEST(GPSHeadingTestSuite, SouthHeading)
{
    double nlat = 39.942136;
    double nlon = -75.19969;
    double slat = 39.942041;
    double slon = -75.199694;
    double heading = Glider::geodetics::gpsHeading(nlat, nlon, slat, slon);

    // we can abs the heading here because -180 and +180 are the same heading
    EXPECT_NEAR(std::abs(heading), M_PI, 0.1);
}

TEST(GPSHeadingTestSuite, EastHeading)
{
    double wlat = 39.942197; 
    double wlon = -75.199524;
    double elat = 39.942193;
    double elon = -75.199374;

    double heading = Glider::geodetics::gpsHeading(wlat, wlon, elat, elon);

    EXPECT_NEAR(heading, M_PI/2, 0.1);
}

TEST(GPSHeadingTestSuite, WestHeading)
{
    double wlat = 39.942197; 
    double wlon = -75.199524;
    double elat = 39.942193;
    double elon = -75.199374;

    double heading = Glider::geodetics::gpsHeading(elat, elon, wlat, wlon);

    // we dont normalize heading, nor should this test it 
    // thus a west heading should be -90 (aka 270).
    EXPECT_NEAR(heading, -M_PI/2, 0.1);
}
