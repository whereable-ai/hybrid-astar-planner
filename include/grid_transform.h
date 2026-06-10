#ifndef GRID_TRANSFORM_H
#define GRID_TRANSFORM_H

#include <cmath>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "constants.h"
#include "helper.h"

namespace HybridAStar {

struct GridPose {
  float x = 0.0f;
  float y = 0.0f;
  float t = 0.0f;
};

class GridTransform {
 public:
  void setOrigin(const geometry_msgs::msg::Pose& origin) {
    origin_ = origin;

    tf2::Quaternion q;
    tf2::fromMsg(origin_.orientation, q);
    yaw_ = tf2::getYaw(q);
    cosYaw_ = std::cos(yaw_);
    sinYaw_ = std::sin(yaw_);
  }

  GridPose toGrid(const geometry_msgs::msg::Pose& pose) const {
    const double dx = pose.position.x - origin_.position.x;
    const double dy = pose.position.y - origin_.position.y;
    const double gridXmeters = cosYaw_ * dx + sinYaw_ * dy;
    const double gridYmeters = -sinYaw_ * dx + cosYaw_ * dy;

    tf2::Quaternion q;
    tf2::fromMsg(pose.orientation, q);

    GridPose gridPose;
    gridPose.x = static_cast<float>(gridXmeters / Constants::cellSize);
    gridPose.y = static_cast<float>(gridYmeters / Constants::cellSize);
    gridPose.t = Helper::normalizeHeadingRad(static_cast<float>(tf2::getYaw(q) - yaw_));
    return gridPose;
  }

  geometry_msgs::msg::Pose toMap(float gridX, float gridY, float gridYaw) const {
    const double xMeters = static_cast<double>(gridX) * Constants::cellSize;
    const double yMeters = static_cast<double>(gridY) * Constants::cellSize;

    geometry_msgs::msg::Pose pose;
    pose.position.x = origin_.position.x + cosYaw_ * xMeters - sinYaw_ * yMeters;
    pose.position.y = origin_.position.y + sinYaw_ * xMeters + cosYaw_ * yMeters;
    pose.position.z = origin_.position.z;

    tf2::Quaternion q;
    q.setRPY(0, 0, gridYaw + yaw_);
    pose.orientation = tf2::toMsg(q);
    return pose;
  }

  double yaw() const { return yaw_; }

 private:
  geometry_msgs::msg::Pose origin_;
  double yaw_ = 0.0;
  double cosYaw_ = 1.0;
  double sinYaw_ = 0.0;
};

}

#endif // GRID_TRANSFORM_H
