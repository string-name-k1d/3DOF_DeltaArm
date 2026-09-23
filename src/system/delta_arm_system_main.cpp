#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "arm/delta_arm_controller.hpp"
#include "arm/motor_driver_node.hpp"

/**
 * @brief Single-process entry point.
 *
 * Registers the delta-arm controller node and the FashionStart motor driver
 * node on one executor. Keeping them in the same process gives tight coupling
 * between the ROS application and the serial-bus hardware (shared DDS
 * intra-process transport, shared executor, no IPC overhead).
 *
 * The control augmenter (per-limb offsets, feedforward, slew clamp) lives
 * INSIDE the controller as a logic-only class (see control_augmenter.hpp) and
 * is applied to every MotorTargets message before it is published, so the
 * driver consumes the already-augmented stream on `arm/motor_targets`
 * directly — there is no separate relay node or arm/motor_targets_cmd topic:
 *
 *   controller (IK output)
 *        |  control augmenter applied in publishTargets()
 *        v
 *   arm/motor_targets --> arm_motor_driver
 *
 * The pipeline is an identity passthrough by default; configure it under
 * `arm_controller` in config/arm_params.yaml (offset_deg / enable_feedforward
 * / max_delta_deg).
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options;

  auto controller = std::make_shared<DeltaArmRos::DeltaArmControllerNode>(options);
  auto driver = std::make_shared<DeltaArmDriverNode::MotorDriverNode>(options);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(controller);
  executor->add_node(driver);

  RCLCPP_INFO(rclcpp::get_logger("arm_system"),
              "arm_system up: controller (+ control augmenter) + motor_driver in one process");
  executor->spin();

  rclcpp::shutdown();
  return 0;
}