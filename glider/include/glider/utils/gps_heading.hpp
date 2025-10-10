/*!
 * Jason Hughes
 * June 2025
 *
 * This header only file implements helpful 
 * utility functions for calculating heading from 
 * differential GPS and converting that to the
 * appropiate frame.
*/

#include <cmath>

namespace Glider
{
namespace geodetics
{

/*! @brief calculate the heading between two GPS points
 *  FROM (lat1, lon1) TO (lat2, lon2)
 *  @param lat1: the FROM latitude in degrees decimal
 *  @param lon1: the FROM longitude in degrees decimal
 *  @param lat2: the TO latitude in degrees decimal
 *  @param lon2: the TO longitude in degrees decimal
 *  @return heading_rad: the heading in radians in the NED frame 
*/
double gpsHeading(double lat1, double lon1, double lat2, double lon2)
{
    double lat1_rad = lat1 * M_PI / 180.0;
    double lon1_rad = lon1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double lon2_rad = lon2 * M_PI / 180.0;

    double lon_diff = lon2_rad - lon1_rad;

    double y = std::sin(lon_diff) * std::cos(lat2_rad);
    double x = std::cos(lat1_rad) * std::sin(lat2_rad) - std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(lon_diff);

    double heading_rad = std::atan2(y,x);

    return heading_rad;
}

/*! @brief convert a heading in radians to degrees and normalize to [0,360)
 *  @param heading: the heading in radians, can be ENU or NED frame
 *  @return heading_deg: normalized heading in degrees in the same frame
 *  that was input
*/
double headingRadiansToDegrees(double heading)
{
    double heading_deg = heading * (180.0 / M_PI);
    
    heading_deg = std::fmod(std::fmod(heading_deg, 360.0) + 360.0, 360.0);
    return heading_deg;
}

/*! @brief convert a heading from the geodetic frame (NED) to the ENU frame
 *  @param geodetic_heading: heading in radians in the geodetic (NED) frame
 *  @return enu_heading: heading in radians in the ENU frame
*/
double geodeticToENU(double geodetic_heading)
{
    double enu_heading = std::fmod((M_PI/2 - geodetic_heading + (2*M_PI)), (2*M_PI));
    
    return enu_heading;
}
} // namespace geodetics
} // namespace glider
