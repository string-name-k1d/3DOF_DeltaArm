#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "arm/motor_driver_node.hpp"

/**
 * @brief Standalone motor-driver entry point.
 *
 * Normally the driver is combined with the controller into the single-process
 * "arm_system" executable for tight coupling. This standalone entry
 * point is provided for cases where you want to run the hardware driver on its
 * own (e.g. testing the driver against real servos, or a custom deployment).
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