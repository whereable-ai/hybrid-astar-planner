#include <cstring>
#include <iostream>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "constants.h"
#include "planner.h"

template<typename T, typename T1>
void message(const T& msg, T1 val = T1()) {
  if (!val) {
    std::cout << "### " << msg << std::endl;
  } else {
    std::cout << "### " << msg << val << std::endl;
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("a_star");

  std::string defaultConfigFile = "config/config.yaml";
  try {
    defaultConfigFile = ament_index_cpp::get_package_share_directory("hybrid_astar") + "/config/config.yaml";
  } catch (const std::exception& ex) {
    RCLCPP_WARN(node->get_logger(), "Could not resolve package config path: %s", ex.what());
  }

  node->declare_parameter<std::string>("config_file", defaultConfigFile);
  const auto configFile = node->get_parameter("config_file").as_string();
  std::string configError;
  if (HybridAStar::Constants::loadFromYaml(configFile, &configError)) {
    RCLCPP_INFO(node->get_logger(), "Loaded Hybrid A* config: %s", configFile.c_str());
  } else {
    HybridAStar::Constants::updateDerivedConstants();
    RCLCPP_WARN(
      node->get_logger(),
      "Could not load Hybrid A* config '%s': %s. Using compiled defaults.",
      configFile.c_str(),
      configError.c_str());
  }

  message<std::string, int>("Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");
  message("cell size: ", HybridAStar::Constants::cellSize);

  if (HybridAStar::Constants::manual) {
    message("mode: ", "manual");
  } else {
    message("mode: ", "auto");
  }

  HybridAStar::Planner hy(node);
  hy.plan();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
