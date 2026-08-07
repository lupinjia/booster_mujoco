#pragma once

#include <iostream>
#include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>

namespace param
{

inline struct SimulationConfig
{
    std::string robot;
    std::filesystem::path robot_scene;

    int domain_id;
    std::string interface;

    int use_joystick;
    std::string joystick_type;
    std::string joystick_device;
    int joystick_bits;

    int print_scene_information;

    int enable_elastic_band;
    int band_attached_link = 0;

    // depth config
    int depth_width;
    int depth_height;
    int crop_left;
    float far_clip;
    float near_clip;
    float depth_dt;
    int enable_depth;
    int visualize_opencv;
    // depth noise config
    int add_depth_stereo_noise;
    float far_noise;
    float near_noise;
    float stereo_full_block_artifacts_prob;
    std::array<float, 2> stereo_full_block_height_mean_std;
    std::array<float, 2> stereo_full_block_width_mean_std;
    std::vector<float> stereo_full_block_values;
    float stereo_half_block_spark_prob;
    float stereo_half_block_value;
    int add_sky_artifacts;
    float sky_artifacts_prob;
    float sky_artifacts_far_distance;
    std::vector<float> sky_artifacts_values;
    std::array<float, 2> sky_artifacts_height_mean_std;
    std::array<float, 2> sky_artifacts_width_mean_std;

    void load_from_yaml(const std::string &filename)
    {
        auto cfg = YAML::LoadFile(filename);
        try
        {
            robot = cfg["robot"].as<std::string>();
            robot_scene = cfg["robot_scene"].as<std::string>();
            domain_id = cfg["domain_id"].as<int>();
            interface = cfg["interface"].as<std::string>();
            use_joystick = cfg["use_joystick"].as<int>();
            joystick_type = cfg["joystick_type"].as<std::string>();
            joystick_device = cfg["joystick_device"].as<std::string>();
            joystick_bits = cfg["joystick_bits"].as<int>();
            print_scene_information = cfg["print_scene_information"].as<int>();
            enable_elastic_band = cfg["enable_elastic_band"].as<int>();
            depth_width = cfg["depth_width"].as<int>();
            depth_height = cfg["depth_height"].as<int>();
            crop_left = cfg["crop_left"].as<int>();
            far_clip = cfg["far_clip"].as<float>();
            near_clip = cfg["near_clip"].as<float>();
            depth_dt = cfg["depth_dt"].as<float>();
            enable_depth = cfg["enable_depth"].as<int>();
            visualize_opencv = cfg["visualize_opencv"].as<int>();
            // depth noise
            add_depth_stereo_noise = cfg["add_depth_stereo_noise"].as<int>();
            far_noise = cfg["far_noise"].as<float>();
            near_noise = cfg["near_noise"].as<float>();
            stereo_full_block_artifacts_prob = cfg["stereo_full_block_artifacts_prob"].as<float>();
            stereo_full_block_height_mean_std = cfg["stereo_full_block_height_mean_std"].as<std::array<float, 2>>();
            stereo_full_block_width_mean_std = cfg["stereo_full_block_width_mean_std"].as<std::array<float, 2>>();
            stereo_full_block_values = cfg["stereo_full_block_values"].as<std::vector<float>>();
            stereo_half_block_spark_prob = cfg["stereo_half_block_spark_prob"].as<float>();
            stereo_half_block_value = cfg["stereo_half_block_value"].as<float>();
            add_sky_artifacts = cfg["add_sky_artifacts"].as<int>();
            sky_artifacts_prob = cfg["sky_artifacts_prob"].as<float>();
            sky_artifacts_far_distance = cfg["sky_artifacts_far_distance"].as<float>();
            sky_artifacts_values = cfg["sky_artifacts_values"].as<std::vector<float>>();
            sky_artifacts_height_mean_std = cfg["sky_artifacts_height_mean_std"].as<std::array<float, 2>>();
            sky_artifacts_width_mean_std = cfg["sky_artifacts_width_mean_std"].as<std::array<float, 2>>();
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            exit(EXIT_FAILURE);
        }
    }
} config;

/* ---------- Command Line Parameters ---------- */
namespace po = boost::program_options;

//※ This function must be called at the beginning of main() function
inline po::variables_map helper(int argc, char** argv)
{
    po::options_description desc("Booster Mujoco");
    desc.add_options()
        ("help,h", "Show help message")
        ("domain_id,i", po::value<int>(&config.domain_id), "DDS domain ID; -i 0")
        ("network,n", po::value<std::string>(&config.interface), "DDS network interface; -n eth0")
        ("robot,r", po::value<std::string>(&config.robot), "Robot type; -r go2")
        ("scene,s", po::value<std::filesystem::path>(&config.robot_scene), "Robot scene file; -s scene_terrain.xml")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        exit(0);
    }

    return vm;
}

}