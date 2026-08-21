#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "arm_hardware/delta_arm_controller.hpp"

/**
 * @brief Standalone controller entry point.
 *
 * Used by the "simulation" launch: runs ONLY the task-space controller node,
 * without touching the motor-driver / hardware. The arm simulation is provided
 * externally (another workspace/package) and talks to the controller over the
 * same topic/service/action names as in the real-hardware mode, so no code
 * changes are needed between simulation and hardware.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto controller = std::make_shared<DeltaArmRos::DeltaArmControllerNode>(rclcpp::NodeOptions());
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(controller);

  RCLCPP_INFO(rclcpp::get_logger("arm_controller"),
              "controller running (simulation mode - motor driver not started)");
  executor->spin();

  rclcpp::shutdown();
  return 0;
}