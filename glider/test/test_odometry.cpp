/*!
* Jason Hughes
* October 2025
* 
* @brief unit test for the Odometry and 
* OdometryWithCovariance objects
*/

#include <gtest/gtest.h>

#include "glider/core/odometry.hpp"

static const double X = 1.0;
static const double Y = 1.0;
static const double Z = 2.0;

TEST(OdometryTestSuite, TestOrientation)
{
    // test initlization from imu prediction
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0);
    gtsam::Point3 t(X, Y, Z);
    gtsam::Point3 v(0.0, 0.0, 0.0);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    // test orientation
    // test it as rot3
    ASSERT_EQ(odom.getOrientation<gtsam::Rot3>().matrix(), rot.matrix());

    // test it as gtsam quaternion
    gtsam::Quaternion q_gtsam = rot.toQuaternion();
    ASSERT_EQ(odom.getOrientation<gtsam::Quaternion>(), q_gtsam);

    // test it as Eigen Quaterniond
    Eigen::Quaterniond q_eigen(1.0, 0.0, 0.0, 0.0);
    ASSERT_EQ(odom.getOrientation<Eigen::Quaterniond>(), q_eigen);

    // test ot as Eigen Vector4d
    Eigen::Vector4d q_vec(1.0, 0.0, 0.0, 0.0);
    ASSERT_EQ(odom.getOrientation<Eigen::Vector4d>(), q_vec);
}

TEST(OdometryTestSuite, TestPosition)
{ 
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0);
    gtsam::Point3 t(X, Y, Z);
    gtsam::Point3 v(0.0, 0.0, 0.0);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    // test as gtsam point3 object
    ASSERT_EQ(odom.getPosition<gtsam::Point3>(), t);
    
    // test as eigen vector 
    Eigen::Vector3d tev(X, Y, Z);
    ASSERT_EQ(odom.getPosition<Eigen::Vector3d>(), tev);
}

TEST(OdometryTestSuite, TestPose)
{ 
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0);
    gtsam::Point3 t(X, Y, Z);
    gtsam::Point3 v(0.0, 0.0, 0.0);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    // test gtsam pose3
    ASSERT_EQ(odom.getPose<gtsam::Pose3>(), ns.pose());

    // test Eigen Isometry
    Eigen::Isometry3d tpose = Eigen::Isometry3d::Identity();
    tpose.linear() = rot.matrix();
    tpose.translation() = Eigen::Vector3d(X, Y, Z);
    ASSERT_EQ(odom.getPose<Eigen::Isometry3d>(), tpose);

    // test pairs
    // test vector pairs
    Eigen::Vector4d orientv(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d t(X, Y, Z);
    std::pair<Eigen::Vector3d, Eigen::Vector4d> pv = odom.getPose<std::pair<Eigen::Vector3d, Eigen::Vector4d>>();
    ASSERT_EQ(pv.first, t);
    ASSERT_EQ(pv.second, orientv);

    //test vector, quat eigen pair
    Eigen::Quaterniond orientq(1.0, 0.0, 0.0, 0.0);
    std::pair<Eigen::Vector3d, Eigen::Quaterniond> pq = odom.getPose<std::pair<EIgen::Vector3d, Eigen::Quaterniond>>();
    ASSERT_EQ(pq.first, t);
    ASSERT_EQ(pq.second. orientq);
}

TEST(OdometryTestSuite, TestVelocity)
{ 
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0);
    gtsam::Point3 t(X, Y, Z);
    gtsam::Point3 v(0.0, 0.0, 0.0);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    // TODO finish this test 
}
