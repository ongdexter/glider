/*
* Jason Hughes
* July 20205
*
* glider-mono parameters
*/

#include "glider/utils/parameters.hpp"


Glider::Parameters::Parameters(const std::string& path)
{
    try
    {
        YAML::Node config = YAML::LoadFile(path);

        // covaraiances
        accel_cov = config["covariances"]["accelerometer"].as<double>();
        gyro_cov = config["covariances"]["gyroscope"].as<double>();
        integration_cov = config["covariances"]["integration"].as<double>();
        bias_cov = config["covariances"]["bias"].as<double>();
        gps_noise = config["convariances"]["gps"].as<double>();

        // constants
        gravity = config["constants"]["gravity"].as<double>();
        lag_time = config["constants"]["lag_time"].as<double>();
        bias_num_measurements = config["constants"]["bias_num_measurements"].as<int>();

        frame = config["frame"]["imu"].as<std::string>();
        t_imu_gps(0) = config["gps_to_imu"]["x"].as<double>();
        t_imu_gps(1) = config["gps_to_imu"]["y"].as<double>();
        t_imu_gps(2) = config["gps_to_imu"]["z"].as<double>();
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error("Error loading YAML File at : " + path + " : " + std::string(e.what()));
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error("Error parsing YAML configuration at: " + path + " : " + std::string(e.what()));
    }   
}

Glider::Parameters Glider::Parameters::Load(const std::string& path)
{
    Parameters params(path);
    return params;
}
