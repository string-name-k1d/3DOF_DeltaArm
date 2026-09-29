#ifndef arm__ARM_ENDPOINT_NODE_HPP_
#define arm__ARM_ENDPOINT_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "arm/endpoint_module.hpp"
#include "arm/msg/cog_estimate.hpp"
#include "arm/msg/control_state.hpp"
#include "arm/msg/endpoint_state.hpp"
#include "arm/msg/torque_estimate.hpp"

namespace DeltaArmRos
{

/**
 * @brief Node owning the terminal end-effector module.
 *
 * Instantiates the module selected by `module_type` (default "null") and
 * republishes its contributions as topics:
 *   - `arm/endpoint/state`           (EndpointState)
 *   - `arm/endpoint/cog_estimate`     (CogEstimate)
 *   - `arm/endpoint/torque_estimate`  (TorqueEstimate)
 *
 * A `arm/endpoint/attach` (std_srvs/SetBool) service toggles the module's
 * connector state, which is what the CoG / torque estimates key off. A
 * subscription to `arm/control_state` (ControlState) is kept so a future
 * version can gate estimation on the arm's control state; today it only logs
 * transitions.
 */
class ArmEndpointNode : public rclcpp::Node
{
public:
  explicit ArmEndpointNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ArmEndpointNode() override;

private:
  void declareParams();
  void setupPublishers();
  void setupSubscriptions();
  void setupServices();
  void setupTimer();
  void publishOnce();

  std::shared_ptr<EndpointModule> module_;

  rclcpp::Publisher<arm::msg::EndpointState>::SharedPtr state_pub_;
  rclcpp::Publisher<arm::msg::CogEstimate>::SharedPtr cog_pub_;
  rclcpp::Publisher<arm::msg::TorqueEstimate>::SharedPtr torque_pub_;
  rclcpp::Subscription<arm::msg::ControlState>::SharedPtr control_state_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr attach_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string frame_id_;
  double publish_rate_;
  std::string last_control_state_;
};

}  // namespace DeltaArmRos

#endif  // arm__ARM_ENDPOINT_NODE_HPP_