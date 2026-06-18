#include "planner.h"
#include <chrono>
#include <cmath>
#include <tf2/utils.h>

using namespace HybridAStar;

Planner::Planner(rclcpp::Node::SharedPtr n) : n(n), path(n, false), smoothedPath(n, true), visualization(n) {
  pubStart = n->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/start", 1);

  auto mapQos = rclcpp::QoS(1).transient_local().reliable();
  if (Constants::manual) {
    subMap = n->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", mapQos, std::bind(&Planner::setMap, this, std::placeholders::_1));
  } else {
    subMap = n->create_subscription<nav_msgs::msg::OccupancyGrid>("/occ_map", mapQos, std::bind(&Planner::setMap, this, std::placeholders::_1));
  }

  subGoal = n->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1, std::bind(&Planner::setGoal, this, std::placeholders::_1));
  subStart = n->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, std::bind(&Planner::setStart, this, std::placeholders::_1));
  subLocalPose = n->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/local_pose",
    rclcpp::SensorDataQoS(),
    std::bind(&Planner::setLocalPose, this, std::placeholders::_1));
  startPublishTimer = n->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&Planner::publishStartPose, this));

  tfBuffer = std::make_shared<tf2_ros::Buffer>(n->get_clock());
  listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer, n, false);
};

void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    dubinsLookup.resize(Constants::dubinsLookupSize());
    Lookup::dubinsLookup(dubinsLookup.data());
  }
  collisionLookup.resize(Constants::collisionLookupSize());
  Lookup::collisionLookup(collisionLookup.data());
  configurationSpace.initializeLookup();
}

void Planner::setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  if (std::abs(map->info.resolution - Constants::cellSize) > 1e-6f) {
    RCLCPP_WARN(
      n->get_logger(),
      "Map resolution %.4f differs from planner cell_size %.4f. Using map resolution.",
      map->info.resolution,
      Constants::cellSize);
    Constants::cellSize = map->info.resolution;
    Constants::updateDerivedConstants();
    initializeLookups();
  }

  grid = map;
  mapTransform.setOrigin(map->info.origin);
  path.setMapOrigin(map->info.origin);
  smoothedPath.setMapOrigin(map->info.origin);
  visualization.setMapOrigin(map->info.origin);
  configurationSpace.updateGrid(map);

  int height = map->info.height;
  int width = map->info.width;

  if (Constants::smoothing) {
    bool** binMap;
    binMap = new bool*[width];

    for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

    for (int x = 0; x < width; ++x) {
      if (!rclcpp::ok()) {
        for (int i = 0; i < width; i++) {
          delete[] binMap[i];
        }
        delete[] binMap;
        return;
      }
      for (int y = 0; y < height; ++y) {
        binMap[x][y] = map->data[y * width + x] ? true : false;
      }
    }

    voronoiDiagram.initializeMap(width, height, binMap);
    voronoiDiagram.update();
    for (int x = 0; x < width; x++) {
      delete[] binMap[x];
    }
    delete[] binMap;
  }

  if (!Constants::manual && tfBuffer->canTransform("map", "base_link", tf2::TimePointZero)) {
    try {
      transform = tfBuffer->lookupTransform("map", "base_link", tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped basePose;
      basePose.header.frame_id = "map";
      basePose.header.stamp = n->now();
      basePose.pose.position.x = transform.transform.translation.x;
      basePose.pose.position.y = transform.transform.translation.y;
      basePose.pose.position.z = transform.transform.translation.z;
      basePose.pose.orientation = transform.transform.rotation;
      updateStartPose(basePose, "base_link");
      plan();
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(n->get_logger(), "Could not transform map to base_link: %s", ex.what());
    }
  }
}

void Planner::setStart(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header = initial->header;
  pose.pose = initial->pose.pose;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = "map";
  }

  geometry_msgs::msg::PoseStamped mapPose;
  if (transformToMap(pose, mapPose, "initialpose")) {
    updateStartPose(mapPose, "initialpose");
  }
}

bool Planner::transformToMap(
    const geometry_msgs::msg::PoseStamped& input,
    geometry_msgs::msg::PoseStamped& output,
    const char* source) {
  output = input;
  if (!output.header.frame_id.empty() && output.header.frame_id.front() == '/') {
    output.header.frame_id.erase(0, 1);
  }

  if (output.header.frame_id.empty() || output.header.frame_id == "map") {
    output.header.frame_id = "map";
    output.header.stamp = n->now();
    return true;
  }

  try {
    const auto transform = tfBuffer->lookupTransform("map", output.header.frame_id, tf2::TimePointZero);
    tf2::doTransform(input, output, transform);
    output.header.frame_id = "map";
    output.header.stamp = n->now();
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(
      n->get_logger(),
      *n->get_clock(),
      2000,
      "Ignoring %s pose in frame '%s': could not transform to map: %s",
      source,
      input.header.frame_id.c_str(),
      ex.what());
    return false;
  }
}

bool Planner::isOnGrid(const GridPose& pose) const {
  return grid &&
         pose.x >= 0.0f &&
         pose.y >= 0.0f &&
         pose.x < static_cast<float>(grid->info.width) &&
         pose.y < static_cast<float>(grid->info.height);
}

void Planner::updateStartPose(const geometry_msgs::msg::PoseStamped& pose, const char* source) {
  if (!grid) {
    validStart = false;
    RCLCPP_WARN_THROTTLE(
      n->get_logger(),
      *n->get_clock(),
      1000,
      "Waiting for map before validating %s start: map=(%.2f, %.2f, %.1f deg)",
      source,
      pose.pose.position.x,
      pose.pose.position.y,
      Helper::toDeg(tf2::getYaw(pose.pose.orientation)));
    return;
  }

  const GridPose gridPose = mapTransform.toGrid(pose.pose);
  if (isOnGrid(gridPose)) {
    validStart = true;
  } else {
    validStart = false;
    RCLCPP_WARN_THROTTLE(
      n->get_logger(),
      *n->get_clock(),
      1000,
      "Ignoring %s start outside map: map=(%.2f, %.2f, %.1f deg), grid=(%.2f, %.2f, %.1f deg)",
      source,
      pose.pose.position.x,
      pose.pose.position.y,
      Helper::toDeg(tf2::getYaw(pose.pose.orientation)),
      gridPose.x,
      gridPose.y,
      Helper::toDeg(gridPose.t));
    return;
  }

  start.header = pose.header;
  start.pose.pose = pose.pose;

  latestStartPose.header.frame_id = "map";
  latestStartPose.header.stamp = n->now();
  latestStartPose.pose = pose.pose;
  hasLatestStartPose = true;
  publishStartPose();

  if (source == std::string("initialpose")) {
    std::cout << "I am seeing a new start x:" << gridPose.x
              << " y:" << gridPose.y
              << " t:" << Helper::toDeg(gridPose.t)
              << std::endl;
  }
}

void Planner::publishStartPose() {
  if (!hasLatestStartPose) {
    return;
  }

  latestStartPose.header.stamp = n->now();
  pubStart->publish(latestStartPose);
}

void Planner::setLocalPose(const geometry_msgs::msg::PoseStamped::SharedPtr local) {
  geometry_msgs::msg::PoseStamped mapPose;
  if (transformToMap(*local, mapPose, "local_pose")) {
    updateStartPose(mapPose, "local_pose");
  }
}

void Planner::setGoal(const geometry_msgs::msg::PoseStamped::SharedPtr end) {
  geometry_msgs::msg::PoseStamped pose = *end;
  if (pose.header.frame_id.empty()) {
    pose.header.frame_id = "map";
  }

  geometry_msgs::msg::PoseStamped mapPose;
  if (!transformToMap(pose, mapPose, "goal")) {
    return;
  }

  const GridPose gridPose = mapTransform.toGrid(mapPose.pose);
  std::cout << "I am seeing a new goal x:" << gridPose.x
            << " y:" << gridPose.y
            << " t:" << Helper::toDeg(gridPose.t)
            << std::endl;

  if (isOnGrid(gridPose)) {
    validGoal = true;
    goal = mapPose;

    if (Constants::manual) { plan();}

  } else {
    validGoal = false;
    std::cout << "invalid goal x:" << gridPose.x
              << " y:" << gridPose.y
              << " t:" << Helper::toDeg(gridPose.t)
              << std::endl;
  }
}

void Planner::plan() {
  if (validStart && validGoal) {

    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();
    const auto cleanupNodes = [&]() {
      delete [] nodes3D;
      delete [] nodes2D;
    };

    const GridPose goalGridPose = mapTransform.toGrid(goal.pose);
    const Node3D nGoal(goalGridPose.x, goalGridPose.y, goalGridPose.t, 0, 0, nullptr);

    const GridPose startGridPose = mapTransform.toGrid(start.pose.pose);
    Node3D nStart(startGridPose.x, startGridPose.y, startGridPose.t, 0, 0, nullptr);

    if (!configurationSpace.isTraversable(&nStart)) {
      RCLCPP_WARN(
        n->get_logger(),
        "Invalid start: vehicle footprint is in collision at x=%.2f y=%.2f.",
        nStart.getX(),
        nStart.getY());
      cleanupNodes();
      return;
    }

    if (!configurationSpace.isTraversable(&nGoal)) {
      RCLCPP_WARN(
        n->get_logger(),
        "Invalid goal: vehicle footprint is in collision at x=%.2f y=%.2f.",
        nGoal.getX(),
        nGoal.getY());
      cleanupNodes();
      return;
    }

    rclcpp::Time t0 = n->now();

    visualization.clear();
    path.clear();
    smoothedPath.clear();

    const rclcpp::Time tSearchStart = n->now();
    Node3D* nSolution = Algorithm::hybridAStar(
      nStart,
      nGoal,
      nodes3D,
      nodes2D,
      width,
      height,
      configurationSpace,
      dubinsLookup.empty() ? nullptr : dubinsLookup.data(),
      visualization,
      n);
    const rclcpp::Time tSearchEnd = n->now();

    if (nSolution != nullptr) {
      smoother.tracePath(nSolution);
      path.updatePath(smoother.getPath());
      const rclcpp::Time tRawPathEnd = n->now();

      if (Constants::smoothing) {
        smoother.smoothPath(voronoiDiagram);
        smoothedPath.updatePath(smoother.getPath());
      }
      const rclcpp::Time tSmoothEnd = n->now();

      rclcpp::Duration d = tSmoothEnd - t0;
      std::cout << "TIME in ms: " << d.seconds() * 1000 << std::endl;
      std::cout << "TIMING in ms search: " << (tSearchEnd - tSearchStart).seconds() * 1000
                << " raw_path: " << (tRawPathEnd - tSearchEnd).seconds() * 1000
                << " smooth: " << (tSmoothEnd - tRawPathEnd).seconds() * 1000
                << " total_compute: " << d.seconds() * 1000
                << std::endl;

      path.publishPath();
      path.publishPathNodes();
      path.publishPathVehicles();
      if (Constants::smoothing) {
        smoothedPath.publishPath();
        smoothedPath.publishPathNodes();
        smoothedPath.publishPathVehicles();
      }
      if (Constants::publishSearchCosts) {
        visualization.publishNode3DCosts(nodes3D, width, height, depth);
        visualization.publishNode2DCosts(nodes2D, width, height);
      }
    } else {
      RCLCPP_WARN(n->get_logger(), "Hybrid A* failed to find a complete path to the goal.");
    }

    cleanupNodes();

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
