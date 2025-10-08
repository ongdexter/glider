/*
*
* ROS node logic
*/

#include "ros/glider_node.hpp"

using namespace GliderROS;

GliderNode::GliderNode(const rclcpp::NodeOptions& options) : rclcpp::Node("glider_node", options)
{
    // Declare ROS parameters
    declare_parameter("publishers.rate", 10.0);
    declare_parameter("publishers.nav_sat_fix", false);
    declare_parameter("publishers.viz.use", false);
    declare_parameter("publishers.viz.origin_easting", 0.0);
    declare_parameter("publishers.viz.origin_northing", 0.0);

    declare_parameter("subscribers.use_odom", false);

    declare_parameter("path", "");

    // Get parameters
    double freq = this->get_parameter("publishers.rate").as_double();
    RCLCPP_INFO_STREAM(this->get_logger(), "Using prediction rate: " << freq);

    publish_nsf_ = this->get_parameter("publishers.nav_sat_fix").as_bool();
    viz_ = this->get_parameter("publishers.viz.use").as_bool();
    origin_easting_ = this->get_parameter("publishers.viz.origin_easting").as_double();
    origin_northing_ = this->get_parameter("publishers.viz.origin_northing").as_double();

    bool use_odom = this->get_parameter("subscribers.use_odom").as_bool();
    RCLCPP_INFO_STREAM(this->get_logger(), "Fusing Odometry: " << std::boolalpha << use_odom);
    
    std::string path = this->get_parameter("path").as_string();
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Loading graph params from: " << path);
    use_sim_time_ = this->get_clock()->get_clock_type() == RCL_ROS_TIME;

    latest_imu_timestamp_ = 0;

    glider_ = std::make_unique<Glider::Glider>(path);
    current_state_ = Glider::State::Uninitialized();

    imu_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    gps_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Create subscribers with callback groups
    auto imu_sub_options = rclcpp::SubscriptionOptions();
    imu_sub_options.callback_group = imu_group_;
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 20, 
                                                                std::bind(&GliderNode::imuCallback, this, std::placeholders::_1),
                                                                imu_sub_options);
    
    auto gps_sub_options = rclcpp::SubscriptionOptions();
    gps_sub_options.callback_group = gps_group_;
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/gps", 1, 
                                                                      std::bind(&GliderNode::gpsCallback, this, std::placeholders::_1),
                                                                      gps_sub_options);

    auto odom_sub_options = rclcpp::SubscriptionOptions();
    odom_sub_options.callback_group = gps_group_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
                                                                   std::bind(&GliderNode::odomCallback, this, std::placeholders::_1),
                                                                   odom_sub_options);

    if (publish_nsf_)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing NavSatFix msg on /glider/fix");
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/glider/fix", 10);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Publishing Odometry msg on /glider/odom");
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glider/odom", 10);
    }

    if(viz_)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing Odometry Viz message on /glider/odom/viz");
        odom_viz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glider/odom/viz", 10);
    }

    // TODO add in predictor
    //std::chrono::milliseconds d = GliderROS::Conversions::hzToDuration(freq);
    //timer_ = this->create_wall_timer(d, std::bind(&GliderNode::interpolationCallback, this));
    RCLCPP_INFO(this->get_logger(), "GliderNode Initialized");
}

int64_t GliderNode::getTime(const builtin_interfaces::msg::Time& stamp) const
{
   return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nanosec);
}

void GliderNode::interpolationCallback()
{
    if (!initialized_ || !current_state_.isInitialized()) return;
    
    if (latest_imu_timestamp_ <= 0)
    {
        return;
    }
    int64_t timestamp = latest_imu_timestamp_;    
    Glider::Odometry odom;
    try {
        odom = glider_->interpolate(timestamp);
    } 
    catch (const std::exception& e) 
    {
        RCLCPP_WARN(this->get_logger(), "Interpolation error: %s", e.what());
        return;
    }

    if (!odom.isInitialized()) return;
    if (publish_nsf_)
    {
        sensor_msgs::msg::NavSatFix msg = GliderROS::Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(odom);
        // TODO type addCovaraince
        //GliderROS::Conversions::addCovariance<sensor_msgs::msg::NavSatFix>(current_state_, msg);
        gps_pub_->publish(msg);
    }
    else
    {
        nav_msgs::msg::Odometry msg = GliderROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(odom);
        GliderROS::Conversions::addCovariance<nav_msgs::msg::Odometry>(current_state_, msg);
        odom_pub_->publish(msg);

        if (viz_)
        {
            nav_msgs::msg::Odometry viz_msg = msg;
            double x = viz_msg.pose.pose.position.x - origin_easting_;
            double y = viz_msg.pose.pose.position.y - origin_northing_;
            viz_msg.pose.pose.position.x = x;
            viz_msg.pose.pose.position.y = y;

            odom_viz_pub_->publish(viz_msg);
        }
    }
}

void GliderNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    Eigen::Vector3d gyro = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->angular_velocity);
    Eigen::Vector3d accel = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->linear_acceleration);
    Eigen::Vector4d orient = GliderROS::Conversions::rosToEigen<Eigen::Vector4d>(msg->orientation);
    int64_t timestamp = getTime(msg->header.stamp);

    glider_->addIMU(timestamp, accel, gyro, orient);

    latest_imu_timestamp_ = timestamp;
}

void GliderNode::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    Eigen::Vector3d gps = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(*msg);

    int64_t timestamp = getTime(msg->header.stamp);

    glider_->addGPS(timestamp, gps);

    try {        
        current_state_ = glider_->optimize();
    }
    catch (const std::exception& e) 
    {
        RCLCPP_WARN(this->get_logger(), "Optimization error: %s", e.what());
        return;
    }
    if (gps_counter_++ > gps_init_count_)
    {
        if (!initialized_) initialized_ = true;
    }
}

void GliderNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    Eigen::Isometry3d pose = GliderROS::Conversions::rosToEigen<Eigen::Isometry3d>(*msg);
    int64_t timestamp = getTime(msg->header.stamp);
    glider_->addOdom(timestamp, pose);
}

RCLCPP_COMPONENTS_REGISTER_NODE(GliderROS::GliderNode)
