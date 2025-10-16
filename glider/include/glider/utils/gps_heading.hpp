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
double gpsHeading(double lat1, double lon1, double lat2, double lon2);
/*! @brief convert a heading in radians to degrees and normalize to [0,360)
 *  @param heading: the heading in radians, can be ENU or NED frame
 *  @return heading_deg: normalized heading in degrees in the same frame
 *  that was input
*/
double headingRadiansToDegrees(double heading);
/*! @brief convert a heading from the geodetic frame (NED) to the ENU frame
 *  @param geodetic_heading: heading in radians in the geodetic (NED) frame
 *  @return enu_heading: heading in radians in the ENU frame
*/
double geodeticToENU(double geodetic_heading);
} // namespace geodetics
} // namespace glider
