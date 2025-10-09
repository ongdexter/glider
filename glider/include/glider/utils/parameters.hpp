/*
* Jason Hughes
* July 2025
*
* glider-mono parameters loaded from a yaml file.
*/

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <string>

namespace Glider
{

struct Parameters
{
    Parameters() = default;
    Parameters(const std::string& path);
    static Parameters Load(const std::string& path);

    double accel_cov;
    double gyro_cov;
    double heading_cov;
    double roll_pitch_cov;
    double integration_cov;
    double bias_cov;
    double gps_noise;

    double gravity;
    double lag_time; 
    int bias_num_measurements;
    uint64_t initial_num_measurements;

    bool log;

    std::string frame;

    Eigen::Vector3d t_imu_gps;
};
}
