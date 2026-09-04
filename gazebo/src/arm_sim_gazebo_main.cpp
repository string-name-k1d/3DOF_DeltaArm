#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "arm_gazebo/arm_sim_gazebo.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmGazebo::ArmSimGazeboNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
