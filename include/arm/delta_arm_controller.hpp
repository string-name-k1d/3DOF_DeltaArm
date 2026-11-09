#ifndef arm__DELTA_ARM_CONTROLLER_HPP_
#define arm__DELTA_ARM_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arm/action/set_position.hpp"
#include "arm/arm_state_control.hpp"
#include "arm/control_augmenter.hpp"
#include "arm/endpoint_module.hpp"
#include "arm/msg/arm_position.hpp"
#include "arm/msg/cog_estimate.hpp"
#include "arm/msg/control_state.hpp"
#include "arm/msg/endpoint_state.hpp"
#include "arm/msg/motor_targets.hpp"
#include "arm/msg/torque_estimate.hpp"
#include "arm/srv/emergency_stop.hpp"
#include "arm/srv/toggle_position_stream.hpp"
#include "arm/delta_arm.hpp"

namespace DeltaArmRos
{

/**
 * @brief Task-space interface to the 3-DOF delta arm.
 *
 *  * set_pos : "arm/set_pos" action. Goal is a Twist (linear.x/y/z).
 *              Runs two-stage IK -> publishes motor targets for the driver.
 *  * get_pos : "arm/pos" topic, published continuously once enabled via
 *              the "arm/get_pos" service (TogglePositionStream).
 *
 * The node also owns the terminal end-effector module (formerly the separate
 * arm_endpoint node) and publishes "arm/endpoint/state" plus the
 * "arm/endpoint/cog_estimate" / "arm/endpoint/torque_estimate" estimates.
 * That keeps the whole arm-side contract on one node: the endpoint data is
 * produced in the same process that owns the kinematics, so it can use the
 * live pose without a cross-node subscription.
 */
class DeltaArmControllerNode : public rclcpp::Node
{
public:
  using SetPosition = arm::action::SetPosition;
  using GoalHandle = rclcpp_action::ServerGoalHandle<SetPosition>;

  explicit DeltaArmControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DeltaArmControllerNode() override;

private:
  void declareParams();
  void setupActionServer();
  void setupGetPosition();
  void setupEmergencyStop();
  void setupEndpoint();
  void publishPosition();
  void publishTargets(const float * angles);
  void publishControlState();
  void publishEndpoint();
  bool applyEmergencyStop(const std::string & reason);
  bool releaseEmergencyStop(const std::string & reason);

  // rclcpp_action callbacks
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SetPosition::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void executeAction(const std::shared_ptr<GoalHandle> goal_handle);

  rclcpp_action::Server<SetPosition>::SharedPtr action_server_;
  rclcpp::Publisher<arm::msg::MotorTargets>::SharedPtr motor_pub_;
  rclcpp::Publisher<arm::msg::ArmPosition>::SharedPtr pos_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<arm::msg::ControlState>::SharedPtr control_state_pub_;
  rclcpp::Service<arm::srv::TogglePositionStream>::SharedPtr toggle_srv_;
  rclcpp::Service<arm::srv::EmergencyStop>::SharedPtr estop_srv_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;
  rclcpp::TimerBase::SharedPtr control_state_timer_;

  // Terminal end-effector module (merged from the former arm_endpoint node).
  // Observational only: it contributes connection state, payload mass/CoG and
  // the CoG / torque estimates; it never commands the motors.
  std::shared_ptr<EndpointModule> endpoint_module_;
  rclcpp::Publisher<arm::msg::EndpointState>::SharedPtr endpoint_state_pub_;
  rclcpp::Publisher<arm::msg::CogEstimate>::SharedPtr endpoint_cog_pub_;
  rclcpp::Publisher<arm::msg::TorqueEstimate>::SharedPtr endpoint_torque_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr endpoint_attach_srv_;
  rclcpp::TimerBase::SharedPtr endpoint_timer_;
  double endpoint_publish_rate_;

  std::shared_ptr<GoalHandle> current_goal_handle_;

  DeltaArm::Arm arm_;

  // Control augmentation (offset / feedforward / slew clamp) is applied to
  // every published MotorTargets by publishTargets(). See control_augmenter.hpp.
  ControlAugmenter augmenter_;

  // High-level control state (zero/emergency, home, position control). Decides
  // whether set_pos goals are accepted; see arm_state_control.hpp.
  ArmStateControl state_control_;

  double feedback_rate_;
  std::string feedback_frame_;
  bool streaming_;
  bool simulate_arrival_;
  bool enable_motion_;
};

}  // namespace DeltaArmRos

#endif  // arm__DELTA_ARM_CONTROLLER_HPP_