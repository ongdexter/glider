/*!
* Jason Hughes
* October 2025
*
*/

#include "glider/core/differential_gps.hpp"

using namespace Glider::Geodetics;

DifferentialGpsFromMotion::DifferentialGpsFromMotion(const std::string& frame, const double& threshold)
{
    frame_ = frame;
    vel_threshold_ = threshold;
}

double DifferentialGpsFromMotion::getHeading(const Eigen::Vector3d& gps)
{
    if (!initialized_)
    {
        last_gps_ = gps;
        initialized_ = true;
    }

    double lat1_rad = last_gps_(0) * M_PI / 180.0;
    double lon1_rad = last_gps_(1) * M_PI / 180.0;
    double lat2_rad = gps(0) * M_PI / 180.0;
    double lon2_rad = gps(1) * M_PI / 180.0;

    double lon_diff = lon2_rad - lon1_rad;

    double y = std::sin(lon_diff) * std::cos(lat2_rad);
    double x = std::cos(lat1_rad) * std::sin(lat2_rad) - std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(lon_diff);

    double heading_rad = std::atan2(y,x);
    last_gps_ = gps;

    if (frame_ == "enu") heading_rad = nedToEnu(heading_rad);

    return heading_rad;
}

bool DifferentialGpsFromMotion::isInitialized() const
{
    return initialized_;
}

double DifferentialGpsFromMotion::getVelocityThreshold() const
{
    return vel_threshold_;
}

void DifferentialGpsFromMotion::setLastGps(const Eigen::Vector3d& gps)
{
    last_gps_ = gps;
    initialized_ = true;
}

double DifferentialGpsFromMotion::nedToEnu(const double heading) const
{
    double enu_heading = std::fmod((M_PI/2 - heading + (2*M_PI)), (2*M_PI));
    return enu_heading;
}

double DifferentialGpsFromMotion::headingRadiansToDegrees(const double heading) const
{
    double heading_deg = heading * (180.0 / M_PI);

    heading_deg = std::fmod(std::fmod(heading_deg, 360.0) + 360.0, 360.0);
    return heading_deg;
}

bool DifferentialGpsFromMotion::isIntegratable(const Eigen::Vector3d& velocity) const
{
    double vel = velocity.head(2).norm();
    return vel > vel_threshold_;
}
