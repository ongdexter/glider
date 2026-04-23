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
    declare_parameter("publishers.utm_zone", "14R");
    declare_parameter("publishers.map_frame", "map");
    declare_parameter("publishers.base_link_frame", "base_link"); 

    declare_parameter("subscribers.dgps_topic", "/dgps");
    declare_parameter("subscribers.use_odom", false);

    declare_parameter("path", "");

    // Get parameters
    freq_ = this->get_parameter("publishers.rate").as_double();

    publish_nsf_ = this->get_parameter("publishers.nav_sat_fix").as_bool();
    viz_ = this->get_parameter("publishers.viz.use").as_bool();
    origin_easting_ = this->get_parameter("publishers.viz.origin_easting").as_double();
    origin_northing_ = this->get_parameter("publishers.viz.origin_northing").as_double();
    map_frame_ = this->get_parameter("publishers.map_frame").as_string();
    base_link_frame_ = this->get_parameter("publishers.base_link_frame").as_string();

    use_odom_ = this->get_parameter("subscribers.use_odom").as_bool();
    
    std::string path = this->get_parameter("path").as_string();

    glider_ = std::make_unique<Glider::Glider>(path);
    utm_zone_ = this->get_parameter("publishers.utm_zone").as_string();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    current_state_ = Glider::OdometryWithCovariance::Uninitialized();

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

    auto dgps_topic = this->get_parameter("subscribers.dgps_topic").as_string();
    dgps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(dgps_topic, rclcpp::SensorDataQoS(),
                                                                 std::bind(&GliderNode::dgpsCallback, this, std::placeholders::_1),
                                                                 gps_sub_options);

    gps_goal_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("/glider/gps_goal", 1, 
                                                                               std::bind(&GliderNode::gpsGoalCallback, this, std::placeholders::_1));

    auto odom_sub_options = rclcpp::SubscriptionOptions();
    odom_sub_options.callback_group = gps_group_;
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
                                                                   std::bind(&GliderNode::odomCallback, this, std::placeholders::_1),
                                                                   odom_sub_options);

    LOG(INFO) << "[GLIDER] Publishing Odometry msg on /glider/odom";
    LOG(INFO) << "[GLIDER] Using prediction rate: " << freq_;
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glider/odom", 10);

    if (publish_nsf_)
    {
        LOG(INFO) << "[GLIDER] Publishing NavSatFix msg on /glider/fix";
        gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/glider/fix", 10);
    }

    if(viz_)
    {
        LOG(INFO) << "[GLIDER] Publishing Odometry Viz message on /glider/odom/viz";
        odom_viz_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/glider/odom/viz", 10);
    }

    LOG(INFO) << "[GLIDER] Publishing Goal Pose on /goal_pose";
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    if (freq_ > 0)
    {
        std::chrono::milliseconds d = GliderROS::Conversions::hzToDuration(freq_);
        timer_ = this->create_wall_timer(d, std::bind(&GliderNode::interpolationCallback, this));
    }
    LOG(INFO) << "[GLIDER] GliderNode Initialized";
}

int64_t GliderNode::getTime(const builtin_interfaces::msg::Time& stamp) const
{
   return (static_cast<int64_t>(stamp.sec) * 1000000000LL) + static_cast<int64_t>(stamp.nanosec);
}

void GliderNode::interpolationCallback()
{
    // if the state is not initialized we cannot interpolate
    if (!current_state_.isInitialized()) return;
    int64_t timestamp = getTime(this->now());
    Glider::Odometry odom = glider_->interpolate(timestamp);

    if (publish_nsf_) publishNavSatFix(odom);
    publishOdometry(odom);
}

void GliderNode::imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Recieved IMU measurement";
    Eigen::Vector3d gyro = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->angular_velocity);
    Eigen::Vector3d accel = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(msg->linear_acceleration);
    Eigen::Vector4d orient = GliderROS::Conversions::rosToEigen<Eigen::Vector4d>(msg->orientation);
    int64_t timestamp = getTime(msg->header.stamp);

    glider_->addImu(timestamp, accel, gyro, orient);

    if (freq_ == 0 && current_state_.isInitialized())
    {
        Glider::Odometry odom = glider_->interpolate(timestamp);
        if (publish_nsf_) publishNavSatFix(odom);
        publishOdometry(odom);
    }
}

void GliderNode::dgpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX || msg->position_covariance[0] < 1e-6)
    {
        LOG_FIRST_N(WARNING, 5) << "[GLIDER] DGPS ignored: No fix or invalid covariance";
        return;
    }

    if (msg->position_covariance[0] > glider_->params().dgps_rejection_limit)
    {
        LOG_FIRST_N(INFO, 1) << "[GLIDER] DGPS rejected due to high covariance (> " << glider_->params().dgps_rejection_limit << ")";
        return;
    }
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Received DGPS measurement";
    Eigen::Vector3d gps = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(*msg);
    int64_t timestamp = getTime(msg->header.stamp);

    double sigma = std::sqrt(msg->position_covariance[0]);
    glider_->addGps(timestamp, gps, sigma); 
    
    current_state_ = glider_->optimize(timestamp);
}

void GliderNode::gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    if (msg->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX || msg->position_covariance[0] < 1e-6)
    {
        LOG_FIRST_N(WARNING, 5) << "[GLIDER] GPS ignored: No fix or invalid covariance";
        return;
    }

    if (msg->position_covariance[0] > glider_->params().dgps_rejection_limit) 
    {
        LOG_FIRST_N(INFO, 1) << "[GLIDER] GPS rejected due to high covariance (> " << glider_->params().dgps_rejection_limit << ")";
        return;
    }
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Recieved GPS measurement";
    Eigen::Vector3d gps = GliderROS::Conversions::rosToEigen<Eigen::Vector3d>(*msg);

    int64_t timestamp = getTime(msg->header.stamp);

    double sigma = std::sqrt(msg->position_covariance[0]);
    glider_->addGps(timestamp, gps, sigma);

    current_state_ = glider_->optimize(timestamp);
}

void GliderNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    if (!use_odom_) return;
    Eigen::Isometry3d pose = GliderROS::Conversions::rosToEigen<Eigen::Isometry3d>(*msg);
    int64_t timestamp = getTime(msg->header.stamp);
    if (glider_->addOdom(timestamp, pose)) 
    {
        current_state_ = glider_->optimize(timestamp);
    }
}

void GliderNode::gpsGoalCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
    if (!glider_->isGpsOffsetInitialized()) 
    {
        LOG_FIRST_N(WARNING, 1) << "[GLIDER] GPS Goal ignored: System not initialized with global origin yet.";
        return;
    }

    LOG(INFO) << "[GLIDER] Received GPS Goal: " << msg->latitude << ", " << msg->longitude;
    
    // Convert GPS Goal (lat, lon, alt) to UTM map coordinates
    double easting, northing;
    char zone[10];
    Glider::geodetics::LLtoUTM(msg->latitude, msg->longitude, northing, easting, zone);
    Eigen::Vector3d goal_utm(easting, northing, 0.0);
    
    // Subtract the GPS offset so the goal is in the same frame as the optimizer output.
    // Outdoor: offset is (0,0,0) so this is a no-op.
    // Indoor (odom-seeded): offset bridges UTM → local frame.
    Eigen::Vector3d gps_offset = glider_->getGpsOffset();
    Eigen::Vector3d goal_local = goal_utm - gps_offset;

    LOG(INFO) << "[GLIDER] GPS Goal in map frame: " << goal_local(0) << ", " << goal_local(1);

    // Publish as a local map frame goal pose
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = map_frame_;
    goal_msg.pose.position.x = goal_local(0);
    goal_msg.pose.position.y = goal_local(1);
    goal_msg.pose.position.z = 0.0;
    goal_msg.pose.orientation.w = 1.0; // Default orientation

    goal_pub_->publish(goal_msg);
}

void GliderNode::publishOdometry(Glider::OdometryWithCovariance& state) const
{
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Publishing Odometry from optimzation";
    nav_msgs::msg::Odometry msg = GliderROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(state, map_frame_);
    msg.child_frame_id = base_link_frame_;
    odom_pub_->publish(msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg.header;
    tf.child_frame_id = msg.child_frame_id;
    tf.transform.translation.x = msg.pose.pose.position.x;
    tf.transform.translation.y = msg.pose.pose.position.y;
    tf.transform.translation.z = msg.pose.pose.position.z;
    tf.transform.rotation = msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
    
    if (viz_) publishOdometryViz(msg);
}

void GliderNode::publishOdometry(Glider::Odometry& odom) const
{
    nav_msgs::msg::Odometry msg = GliderROS::Conversions::odomToRos<nav_msgs::msg::Odometry>(odom, map_frame_);
    msg.child_frame_id = base_link_frame_;
    GliderROS::Conversions::addCovariance<nav_msgs::msg::Odometry>(current_state_, msg);
    odom_pub_->publish(msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = msg.header;
    tf.child_frame_id = msg.child_frame_id;
    tf.transform.translation.x = msg.pose.pose.position.x;
    tf.transform.translation.y = msg.pose.pose.position.y;
    tf.transform.translation.z = msg.pose.pose.position.z;
    tf.transform.rotation = msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    if (viz_) publishOdometryViz(msg);
}

void GliderNode::publishNavSatFix(Glider::OdometryWithCovariance& state) const
{
    if (!glider_->isGpsInitialized()) return;

    // TODO add covariance
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Publishing NavSatFix from optimization";
    Eigen::Vector3d offset = glider_->getGpsOffset();
    if (offset.norm() == 0.0 && origin_easting_ != 0.0) {
        offset(0) = origin_easting_;
        offset(1) = origin_northing_;
    }
    std::string zone = glider_->getUtmZone();
    if (zone.empty()) zone = utm_zone_;
    sensor_msgs::msg::NavSatFix msg = GliderROS::Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(state, base_link_frame_, zone.c_str(), offset);
    GliderROS::Conversions::addCovariance<sensor_msgs::msg::NavSatFix>(current_state_, msg);
    gps_pub_->publish(msg);
}

void GliderNode::publishNavSatFix(Glider::Odometry& odom) const
{
    if (!glider_->isGpsInitialized()) return;

    // TODO add covariance
    LOG_FIRST_N(INFO, 1) << "[GLIDER] Publishing NavSatFix from prediction";
    Eigen::Vector3d offset = glider_->getGpsOffset();
    if (offset.norm() == 0.0 && origin_easting_ != 0.0) {
        offset(0) = origin_easting_;
        offset(1) = origin_northing_;
    }
    sensor_msgs::msg::NavSatFix msg = GliderROS::Conversions::odomToRos<sensor_msgs::msg::NavSatFix>(odom, base_link_frame_, utm_zone_.c_str(), offset);
    GliderROS::Conversions::addCovariance<sensor_msgs::msg::NavSatFix>(current_state_, msg);
    gps_pub_->publish(msg);
}

void GliderNode::publishOdometryViz(nav_msgs::msg::Odometry viz_msg) const
{ 
    double x = viz_msg.pose.pose.position.x - origin_easting_;
    double y = viz_msg.pose.pose.position.y - origin_northing_;
    viz_msg.pose.pose.position.x = x;
    viz_msg.pose.pose.position.y = y;

    odom_viz_pub_->publish(viz_msg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(GliderROS::GliderNode)
