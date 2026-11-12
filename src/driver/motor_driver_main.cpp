#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "arm/motor_driver_node.hpp"

/**
 * @brief Motor-driver entry point.
 *
 * Runs the hardware driver as its own process. The controller is launched
 * separately and talks to it over arm/motor_targets and arm/MotorFeedback.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto driver = std::make_shared<DeltaArmDriverNode::MotorDriverNode>(rclcpp::NodeOptions());
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(driver);

  RCLCPP_INFO(rclcpp::get_logger("arm_motor_driver"), "motor driver node running");
  executor->spin();

  rclcpp::shutdown();
  return 0;
}