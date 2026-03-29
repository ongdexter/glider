/*!
* Jason Hughes
* January 2026 
*
*
* Struct to track landmark points
*/

#pragma once

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

namespace Glider
{
class Point
{
    public:
        Point() = default;
        Point(const Eigen::Vector3d& p);
        Point(double x, double y, double z);    

        double x() const { return x_; }
        double y() const { return y_; }
        double z() const { return z_; }
    
        void x(double x) { x_ = x; }
        void y(double y) { y_ = y; }
        void z(double z) { z_ = z; }

        Eigen::Vector3d vector() const;

        bool isInitialized() const { return initialized_; }

    private:
        bool initialized_{false};
        double x_{0.0}, y_{0.0}, z_{0.0};
};

class PointWithCovariance : public Point
{
    public:
        using Point::Point;
        PointWithCovariance() = default;
        PointWithCovariance(const Eigen::Vector3d& p, const Eigen::Matrix3d& c);

        Eigen::Matrix3d cov() const;
        double cov(int r, int c) const;

        Eigen::Vector3d var() const;
        double var(int i) const;

        Eigen::Vector3d std() const;
        double std(int i) const;

        double trace() const;

    private:
        Eigen::Matrix3d cov_{Eigen::Matrix3d::Identity()}; 
};
} // namespace Glider
