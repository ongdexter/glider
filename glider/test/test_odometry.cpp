/*!
* Jason Hughes
* October 2025
* 
* @brief unit test for the Odometry and 
* OdometryWithCovariance objects
*/

#include <gtest/gtest.h>

#include "glider/core/odometry.hpp"

static const double LATITUDE = 39.941259;
static const double LONGITUDE = -75.199202;
static const char ZONE[4] = "18S";
static const double TOL = 0.0001;

static const double TX = 482981.6098624719;
static const double TY = 4421256.568684888;
static const double TZ = 10.0;

static const double VX = 0.0;
static const double VY = 0.0;
static const double VZ = 0.0;

static const double QW = 1.0;
static const double QX = 0.0;
static const double QY = 0.0;
static const double QZ = 0.0;

TEST(OdometryTestSuite, TestOrientation)
{
    // test initlization from imu prediction
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(QW, QX, QY, QZ);
    gtsam::Point3 t(TX, TY, TZ);
    gtsam::Point3 v(VX, VY, VZ);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    // test orientation
    // test it as rot3
    ASSERT_EQ(odom.getOrientation<gtsam::Rot3>().matrix(), rot.matrix());

    // test it as gtsam quaternion
    gtsam::Quaternion q_gtsam = rot.toQuaternion();
    ASSERT_EQ(odom.getOrientation<gtsam::Quaternion>(), q_gtsam);

    // test it as Eigen Quaterniond
    Eigen::Quaterniond q_eigen(QW, QX, QY, QZ);
    ASSERT_EQ(odom.getOrientation<Eigen::Quaterniond>(), q_eigen);

    // test ot as Eigen Vector4d
    Eigen::Vector4d q_vec(QW, QX, QY, QZ);
    ASSERT_EQ(odom.getOrientation<Eigen::Vector4d>(), q_vec);
}

TEST(OdometryTestSuite, TestPosition)
{ 
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(QW, QX, QY, QZ);
    gtsam::Point3 t(TX, TY, TZ);
    gtsam::Point3 v(VX, VY, VZ);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    // test as gtsam point3 object
    ASSERT_EQ(odom.getPosition<gtsam::Point3>(), t);
    
    // test as eigen vector 
    Eigen::Vector3d tev(TX, TY, TZ);
    ASSERT_EQ(odom.getPosition<Eigen::Vector3d>(), tev);
}

TEST(OdometryTestSuite, TestPose)
{ 
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(QW, QX, QY, QZ);
    gtsam::Point3 t(TX, TY, TZ);
    gtsam::Point3 v(VX, VY, VZ);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    // test gtsam pose3
    gtsam::Pose3 pose = odom.getPose<gtsam::Pose3>();
    ASSERT_EQ(pose.rotation().matrix(), rot.matrix());
    ASSERT_EQ(pose.translation(), t);

    // test Eigen Isometry
    Eigen::Isometry3d tpose = Eigen::Isometry3d::Identity();
    tpose.linear() = rot.matrix();
    tpose.translation() = Eigen::Vector3d(TX, TY, TZ);
    ASSERT_EQ(odom.getPose<Eigen::Isometry3d>().linear(), tpose.linear());
    ASSERT_EQ(odom.getPose<Eigen::Isometry3d>().translation(), tpose.translation());

    // test pairs
    // test vector pairs
    Eigen::Vector4d orientv(QW, QX, QY, QZ);
    Eigen::Vector3d tv(TX, TY, TZ);
    std::pair<Eigen::Vector3d, Eigen::Vector4d> pv = odom.getPose<std::pair<Eigen::Vector3d, Eigen::Vector4d>>();
    ASSERT_EQ(pv.first, tv);
    ASSERT_EQ(pv.second, orientv);

    //test vector, quat eigen pair
    Eigen::Quaterniond orientq(QW, QX, QY, QZ);
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> pq = odom.getPose<std::pair<Eigen::Vector3d, Eigen::Quaterniond>>();
    ASSERT_EQ(pq.first, tv);
    ASSERT_EQ(pq.second, orientq);
}

TEST(OdometryTestSuite, TestVelocity)
{ 
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(QW, QX, QY, QZ);
    gtsam::Point3 t(TX, TY, TZ);
    gtsam::Point3 v(VX, VY, VZ);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    // test velocity getters
    // as gtsam object
    ASSERT_EQ(odom.getVelocity<gtsam::Point3>(), v);

    // as eigen vector
    Eigen::Vector3d vv(VX, VY,VZ);
    ASSERT_EQ(odom.getVelocity<Eigen::Vector3d>(), vv);
}

TEST(OdometryTestSuite, TestLatitudeLongitude)
{ 
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(QW, QX, QY, QZ);
    gtsam::Point3 t(TX, TY, TZ);
    gtsam::Point3 v(VX, VY, VZ);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    double lat = odom.getLatitude(ZONE);
    ASSERT_NEAR(lat, LATITUDE, TOL);

    double lon = odom.getLongitude(ZONE);
    ASSERT_NEAR(lon, LONGITUDE, TOL);

    std::pair<double, double> ll = odom.getLatLon(ZONE);
    ASSERT_EQ(ll.first, lat);
    ASSERT_EQ(ll.second, lon);
}

TEST(OdometryTestSuite, TestHeading)
{ 
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(QW, QX, QY, QZ);
    gtsam::Point3 t(TX, TY, TZ);
    gtsam::Point3 v(VX, VY, VZ);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    double heading_gt = rot.yaw();

    ASSERT_EQ(odom.getHeading(), heading_gt);
}

TEST(OdometryTestSuite, TestAltitude)
{
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(QW, QX, QY, QZ);
    gtsam::Point3 t(TX, TY, TZ);
    gtsam::Point3 v(VX, VY, VZ);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    ASSERT_EQ(odom.getAltitude(), TZ);
}

TEST(OdometryTestSuite, TestInitialization)
{ 
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(QW, QX, QY, QZ);
    gtsam::Point3 t(TX, TY, TZ);
    gtsam::Point3 v(VX, VY, VZ);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    ASSERT_TRUE(odom.isInitialized());

    Glider::Odometry u_odom = Glider::Odometry::Uninitialized();

    ASSERT_FALSE(u_odom.isInitialized());

    Glider::Odometry d_odom;
    ASSERT_FALSE(d_odom.isInitialized());
}

TEST(OdometryTestSuite, TestTimestamp)
{ 
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0);
    gtsam::Point3 t(TX, TY, TZ);
    gtsam::Point3 v(VX, VY, VZ);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    ASSERT_EQ(odom.getTimestamp(), timestamp);
}
