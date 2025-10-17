/*!
*  Jason Hughes
*  October 2025
*
*  Object that handles everything for differenctial gps
*  from motion
*/

#pragma once 

#include <cmath>
#include <string>
#include <Eigen/Dense>

namespace Glider
{
namespace Geodetics
{
class DifferentialGpsFromMotion
{
    public:
        DifferentialGpsFromMotion() = default;
        DifferentialGpsFromMotion(const std::string& frame, const double& threshold);

        double getHeading(const Eigen::Vector3d& gps);

        double headingRadiansToDegrees(const double heading) const;

        void setLastGps(const Eigen::Vector3d& gps);
        bool isInitialized() const;
        double getVelocityThreshold() const;
        bool isIntegratable(const Eigen::Vector3d& vel) const;

    private:

        double nedToEnu(const double heading) const;

        Eigen::Vector3d last_gps_;
        std::string frame_;
        double vel_threshold_;
        bool initialized_{false};
};
} // namespace Geodetics
} // namespace Glider
