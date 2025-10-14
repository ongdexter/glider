/*! 
* Jason Hughes
* October 2025
*
* Unit tests for ros to eigen and
* eigen to ros convertions
*/

#include <gtest/gtest.h>

#include "ros/conversions.hpp"

TEST(RosToEigenTestSuite, Vector3Test)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = 1.0;
    msg.y = 1.0;
    msg.z = 1.0;

    Eigen::Vector3d gt(msg.x, msg.y, msg.z);

    ASSERT_EQ(GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg), gt);
}

TEST(RosToEigenTestSuite, OrientTest)
{
    geometry_msgs::msg::Quaternion msg;
    msg.w = 1.0;
    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = 0.0;

    Eigen::Vector4d gt(msg.w, msg.x, msg.y, msg.z);

    ASSERT_EQ(GliderROS::Conversions::rosToEigen<Eigen::Vector4d>(msg), gt);
}

TEST(RosToEigenTestSuite, NavSatFixTest)
{
    sensor_msgs::msg::NavSatFix msg;
    msg.latitude = 32.925;
    msg.longitude = -75.119;
    msg.altitude = 10.0;

    Eigen::Vector3d gt(msg.latitude, msg.longitude, msg.altitude);

    ASSERT_EQ(GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg), gt);
}

TEST(RosToEigenTestSuite, PoseTest)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x = 1.0;
    msg.pose.position.y = 1.0;
    msg.pose.position.z = 1.0;
    msg.pose.orientation.w = 1.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
 
    Eigen::Quaterniond quat(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    Eigen::Vector3d trans(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    Eigen::Isometry3d gt = Eigen::Isometry3d::Identity();
    gt.linear() = quat.toRotationMatrix();
    gt.translation() = trans;

    ASSERT_EQ(GliderROS::Conversions::rosToEigen<Eigen::Isometry3d>(msg).linear(), gt.linear());
    ASSERT_EQ(GliderROS::Conversions::rosToEigen<Eigen::Isometry3d>(msg).translation(), gt.translation());
}

TEST(RosToEigenTestSuite, OdometryTest)
{
    nav_msgs::msg::Odometry msg;
    msg.pose.pose.position.x = 1.0;
    msg.pose.pose.position.y = 1.0;
    msg.pose.pose.position.z = 1.0;
    msg.pose.pose.orientation.w = 1.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = 0.0;
    msg.twist.twist.linear.x = 1.0;
    msg.twist.twist.linear.y = 1.0;
    msg.twist.twist.linear.z = 1.0;

    Eigen::Isometry3d odom = GliderROS::Conversions::rosToEigen<Eigen::Isometry3d>(msg);
 
    Eigen::Quaterniond quat(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Vector3d trans(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

    Eigen::Isometry3d gt = Eigen::Isometry3d::Identity();
    gt.linear() = quat.toRotationMatrix();
    gt.translation() = trans;

    ASSERT_EQ(odom.linear(), gt.linear());
    ASSERT_EQ(odom.translation(), gt.translation());
}

TEST(EigenToRosTestSuite, Vector3Test)
{
    Eigen::Vector3d in(1.0, 1.0, 1.0);

    geometry_msgs::msg::Vector3 msg = GliderROS::Conversions::eigenToRos<geometry_msgs::msg::Vector3>(in);

    ASSERT_EQ(msg.x, in(0));
    ASSERT_EQ(msg.y, in(1));
    ASSERT_EQ(msg.z, in(2));
}

TEST(EigenToRosTestSuite, NavSatFixTest)
{
    Eigen::Vector3d in(32.925, -75.119, 10.0);

    sensor_msgs::msg::NavSatFix msg = GliderROS::Conversions::eigenToRos<sensor_msgs::msg::NavSatFix>(in);

    ASSERT_EQ(msg.latitude, in(0));
    ASSERT_EQ(msg.longitude, in(1));
    ASSERT_EQ(msg.altitude, in(2));
}   

TEST(EigenToRosTestSuite, OrientTest)
{
    Eigen::Vector4d in(1.0, 0.0, 0.0, 0.0);

    geometry_msgs::msg::Quaternion msg = GliderROS::Conversions::eigenToRos<geometry_msgs::msg::Quaternion>(in);

    ASSERT_EQ(msg.w, in(0));
    ASSERT_EQ(msg.x, in(1));
    ASSERT_EQ(msg.y, in(2));
    ASSERT_EQ(msg.z, in(3));
}

TEST(EigenToRosTestSuite, PoseTest)
{
    Eigen::Isometry3d in = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond quat(1.0, 0.0, 0.0, 0.0);
    Eigen::Vector3d t(1.0, 1.0, 1.0);

    in.linear() = quat.toRotationMatrix();
    in.translation() = t;

    geometry_msgs::msg::PoseStamped msg = GliderROS::Conversions::eigenToRos<geometry_msgs::msg::PoseStamped>(in);

    ASSERT_EQ(msg.pose.position.x, t(0));
    ASSERT_EQ(msg.pose.position.y, t(1));
    ASSERT_EQ(msg.pose.position.z, t(2));

    ASSERT_EQ(msg.pose.orientation.w, quat.w());
    ASSERT_EQ(msg.pose.orientation.x, quat.x());
    ASSERT_EQ(msg.pose.orientation.y, quat.y());
    ASSERT_EQ(msg.pose.orientation.z, quat.z());
}

TEST(OdometryToRosTestSuite, OdometryTest)
{
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0);
    gtsam::Point3 t(482981.6098624719, 4421256.568684888, 10.0);
    gtsam::Point3 v(0.0, 0.0, 0.0);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    nav_msgs::msg::Odometry msg = GliderROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(odom);

    ASSERT_EQ(msg.pose.pose.position.x, odom.getPosition<Eigen::Vector3d>()(0));
    ASSERT_EQ(msg.pose.pose.position.y, odom.getPosition<Eigen::Vector3d>()(1));
    ASSERT_EQ(msg.pose.pose.position.z, odom.getPosition<Eigen::Vector3d>()(2));

    ASSERT_EQ(msg.pose.pose.orientation.w, odom.getOrientation<Eigen::Vector4d>()(0));
    ASSERT_EQ(msg.pose.pose.orientation.x, odom.getOrientation<Eigen::Vector4d>()(1));
    ASSERT_EQ(msg.pose.pose.orientation.y, odom.getOrientation<Eigen::Vector4d>()(2));
    ASSERT_EQ(msg.pose.pose.orientation.z, odom.getOrientation<Eigen::Vector4d>()(3));

    ASSERT_EQ(msg.twist.twist.linear.x, odom.getVelocity<Eigen::Vector3d>()(0));
    ASSERT_EQ(msg.twist.twist.linear.y, odom.getVelocity<Eigen::Vector3d>()(1));
    ASSERT_EQ(msg.twist.twist.linear.z, odom.getVelocity<Eigen::Vector3d>()(2));

    ASSERT_STREQ(msg.header.frame_id.c_str(), "enu");
    ASSERT_GT(msg.header.stamp.nanosec, 0);
}

TEST(OdometryToRosTestSuite, NavSatFixTest)
{ 
    int64_t timestamp = 1;
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(1.0, 0.0, 0.0, 0.0);
    gtsam::Point3 t(482981.6098624719, 4421256.568684888, 10.0);
    gtsam::Point3 v(0.0, 0.0, 0.0);

    gtsam::NavState ns(rot, t, v);

    Glider::Odometry odom(ns, timestamp, true);

    sensor_msgs::msg::NavSatFix msg = GliderROS::Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(odom, "18S");

    ASSERT_NEAR(msg.latitude, 39.941259, 0.001);
    ASSERT_NEAR(msg.longitude, -75.199202, 0.001);
    ASSERT_EQ(msg.altitude, 10.0);
    ASSERT_STREQ(msg.header.frame_id.c_str(), "enu");
    ASSERT_GT(msg.header.stamp.nanosec, 0);
}

