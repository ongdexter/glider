/*!
* Jason Hughes
* January 2026
*
* Point and PointWithCovariance obejcts
*/

#include "glider/core/point.hpp"

using namespace Glider;

Point::Point(const Eigen::Vector3d& p)
{
    x_ = p.x();
    y_ = p.y();
    z_ = p.z();
    initialized_ = true;
}

Point::Point(double x, double y, double z)
{
    x_ = x;
    y_ = y;
    z_ = z;
    initialized_ = true;
}

Eigen::Vector3d Point::vector() const
{
    return Eigen::Vector3d(x_, y_, z_);
}

PointWithCovariance::PointWithCovariance(const Eigen::Vector3d& p, const Eigen::Matrix3d& c) : Point(p)
{
    cov_ = c;
}

Eigen::Matrix3d PointWithCovariance::cov() const
{
    return cov_;
}

double PointWithCovariance::cov(int r, int c) const
{
    return cov_(r, c);
}

Eigen::Vector3d PointWithCovariance::var() const
{
    return cov_.diagonal();
}

double PointWithCovariance::var(int i) const
{
    Eigen::Vector3d var = cov_.diagonal();
    return var(i);
}

Eigen::Vector3d PointWithCovariance::std() const
{
    return cov_.diagonal().cwiseSqrt();
}

double PointWithCovariance::std(int i) const
{
    Eigen::Vector3d std = cov_.diagonal().cwiseSqrt();
    return std(i);
}

double PointWithCovariance::trace() const
{
    return cov_.trace();
}
