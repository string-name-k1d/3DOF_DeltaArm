#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "arm/delta_arm_controller.hpp"

/**
 * @brief Controller entry point.
 *
 * Runs the task-space controller as its own process. It publishes motor
 * targets for the separate driver node (or an external simulation) and
 * exposes the same interfaces in both simulation and hardware mode.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<DeltaArmRos::DeltaArmControllerNode>(rclcpp::NodeOptions());
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(controller);

  RCLCPP_INFO(rclcpp::get_logger("arm_controller"), "controller running");
  executor->spin();

  rclcpp::shutdown();
  return 0;
}