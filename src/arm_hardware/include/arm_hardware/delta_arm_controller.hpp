#ifndef arm_HARDWARE__DELTA_ARM_CONTROLLER_HPP_
#define arm_HARDWARE__DELTA_ARM_CONTROLLER_HPP_

#include <memory>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arm_msgs/action/set_position.hpp"
#include "arm_msgs/msg/arm_position.hpp"
#include "arm_msgs/msg/motor_targets.hpp"
#include "arm_msgs/srv/emergency_stop.hpp"
#include "arm_msgs/srv/toggle_position_stream.hpp"
#include "arm_hardware/delta_arm.hpp"

namespace DeltaArmRos
{

/**
 * @brief Task-space interface to the 3-DOF delta arm.
 *
 *  * set_pos : "arm/set_pos" action. Goal is a Twist (linear.x/y/z).
 *              Runs two-stage IK -> publishes motor targets for the driver.
 *  * get_pos : "arm/pos" topic, published continuously once enabled via
 *              the "arm/get_pos" service (TogglePositionStream).
 */
class DeltaArmControllerNode : public rclcpp::Node
{
public:
  using SetPosition = arm_msgs::action::SetPosition;
  using GoalHandle = rclcpp_action::ServerGoalHandle<SetPosition>;

  explicit DeltaArmControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DeltaArmControllerNode() override;

private:
  void declareParams();
  void setupActionServer();
  void setupGetPosition();
  void setupEmergencyStop();
  void publishPosition();
  void publishTargets(const float * angles);

  // rclcpp_action callbacks
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SetPosition::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void executeAction(const std::shared_ptr<GoalHandle> goal_handle);

  rclcpp_action::Server<SetPosition>::SharedPtr action_server_;
  rclcpp::Publisher<arm_msgs::msg::MotorTargets>::SharedPtr motor_pub_;
  rclcpp::Publisher<arm_msgs::msg::ArmPosition>::SharedPtr pos_pub_;
  rclcpp::Service<arm_msgs::srv::TogglePositionStream>::SharedPtr toggle_srv_;
  rclcpp::Service<arm_msgs::srv::EmergencyStop>::SharedPtr estop_srv_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  std::shared_ptr<GoalHandle> current_goal_handle_;

  DeltaArm::Arm arm_;

  double feedback_rate_;
  std::string feedback_frame_;
  bool streaming_;
  bool estop_active_;
};

}  // namespace DeltaArmRos

#endif  // arm_HARDWARE__DELTA_ARM_CONTROLLER_HPP_