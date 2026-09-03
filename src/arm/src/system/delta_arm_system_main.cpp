#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "arm/delta_arm_controller.hpp"
#include "arm/motor_driver_node.hpp"

/**
 * @brief Single-process entry point.
 *
 * Registers both the delta-arm controller node and the FashionStart motor
 * driver node on one executor. Keeping them in the same process gives tight
 * coupling between the ROS application and the serial-bus hardware (shared
 * DDS intra-process transport, shared executor, no IPC overhead).
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options;

  auto controller = std::make_shared<DeltaArmRos::DeltaArmControllerNode>(options);
  auto driver     = std::make_shared<DeltaArmDriverNode::MotorDriverNode>(options);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(controller);
  executor->add_node(driver);

  RCLCPP_INFO(rclcpp::get_logger("arm_system"),
              "arm_system up: controller + motor_driver in one process");
  executor->spin();

  rclcpp::shutdown();
  return 0;
}