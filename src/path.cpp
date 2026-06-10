#include "path.h"

using namespace HybridAStar;

void Path::clear() {
  Node3D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  addNode(node, 0);
  addVehicle(node, 1);
  publishPath();
  publishPathNodes();
  publishPathVehicles();
}

void Path::updatePath(const std::vector<Node3D>& nodePath) {
  path.header.frame_id = "map";
  path.header.stamp = n->now();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i) {
    addSegment(nodePath[i]);
    addNode(nodePath[i], k);
    k++;
    addVehicle(nodePath[i], k);
    k++;
  }
}

void Path::addSegment(const Node3D& node) {
  geometry_msgs::msg::PoseStamped vertex;
  vertex.header.frame_id = "map";
  vertex.header.stamp = n->now();
  vertex.pose = transform.toMap(node.getX(), node.getY(), node.getT());
  path.poses.push_back(vertex);
}

void Path::addNode(const Node3D& node, int i) {
  visualization_msgs::msg::Marker pathNode;

  if (i == 0) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "map";
  pathNode.header.stamp = n->now();
  pathNode.id = i;
  pathNode.type = visualization_msgs::msg::Marker::SPHERE;
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  } else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose = transform.toMap(node.getX(), node.getY(), node.getT());
  pathNodes.markers.push_back(pathNode);
}

void Path::addVehicle(const Node3D& node, int i) {
  visualization_msgs::msg::Marker pathVehicle;

  if (i == 1) {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "map";
  pathVehicle.header.stamp = n->now();
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::msg::Marker::CUBE;
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (smoothed) {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  } else {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose = transform.toMap(node.getX(), node.getY(), node.getT());
  pathVehicles.markers.push_back(pathVehicle);
}
