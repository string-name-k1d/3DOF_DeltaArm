#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "arm/arm_endpoint_node.hpp"
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
 *
 * The same file also registers the terminal end-effector node, which owns the
 * swappable EndpointModule and publishes the endpoint state plus the CoG and
 * torque estimates (see endpoint_module.hpp). It is observational only — it
 * never commands the motors.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options;

  auto controller = std::make_shared<DeltaArmRos::DeltaArmControllerNode>(options);
  auto driver = std::make_shared<DeltaArmDriverNode::MotorDriverNode>(options);
  auto endpoint = std::make_shared<DeltaArmRos::ArmEndpointNode>(options);

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(controller);
  executor->add_node(driver);
  executor->add_node(endpoint);

  RCLCPP_INFO(rclcpp::get_logger("arm_system"),
              "arm_system up: controller (+ control augmenter) + motor_driver + endpoint in one process");
  executor->spin();

  rclcpp::shutdown();
  return 0;
}