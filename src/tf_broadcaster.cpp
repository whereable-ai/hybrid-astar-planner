//###################################################
//                        TF MODULE FOR THE HYBRID A*
//###################################################
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TFBroadcaster : public rclcpp::Node {
public:
  TFBroadcaster() : Node("tf_broadcaster") {
    broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&TFBroadcaster::broadcast_timer_callback, this));
  }

private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  void broadcast_timer_callback() {
    rclcpp::Time now = this->now();
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);

    // Keep compatibility with older visualizations that used the "path" frame.
    // The planner now publishes path and markers directly in "map".
    geometry_msgs::msg::TransformStamped t_map_path;
    t_map_path.header.stamp = now;
    t_map_path.header.frame_id = "map";
    t_map_path.child_frame_id = "path";
    t_map_path.transform.translation.x = 0;
    t_map_path.transform.translation.y = 0;
    t_map_path.transform.translation.z = 0;
    t_map_path.transform.rotation.x = q.x();
    t_map_path.transform.rotation.y = q.y();
    t_map_path.transform.rotation.z = q.z();
    t_map_path.transform.rotation.w = q.w();
    broadcaster_->sendTransform(t_map_path);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
