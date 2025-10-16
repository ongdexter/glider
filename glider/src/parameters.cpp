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
        accel_cov = config["imu"]["covariances"]["accelerometer"].as<double>();
        gyro_cov = config["imu"]["covariances"]["gyroscope"].as<double>();
        heading_cov = config["imu"]["covariances"]["heading"].as<double>();
        roll_pitch_cov = config["imu"]["covariances"]["roll_pitch"].as<double>();
        integration_cov = config["imu"]["covariances"]["integration"].as<double>();
        bias_cov = config["imu"]["covariances"]["bias"].as<double>();
        gps_noise = config["gps"]["covariance"].as<double>();
        
        // constants
        gravity = config["constants"]["gravity"].as<double>();
        bias_num_measurements = config["constants"]["bias_num_measurements"].as<int>();
        initial_num_measurements = config["constants"]["initial_num_measurements"].as<uint64_t>();

        frame = config["imu"]["frame"].as<std::string>();
        
        log = config["logging"]["stdout"].as<bool>(); 
        log_dir = config["logging"]["directory"].as<std::string>();

        smooth = config["optimizer"]["smooth"].as<bool>();
        lag_time = config["optimizer"]["lag_time"].as<double>();

        use_dgpsfm = config["dgpsfm"]["enable"].as<bool>();
        dgpsfm_threshold = config["dgpsfm"]["integration_threshold"].as<double>();
        dgpsfm_cov = config["dgpsfm"]["covariance"].as<double>();

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
