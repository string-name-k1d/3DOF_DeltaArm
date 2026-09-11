#include "arm/delta_arm_controller.hpp"

#include <chrono>
#include <functional>
#include <vector>

namespace DeltaArmRos
{

DeltaArmControllerNode::DeltaArmControllerNode(const rclcpp::NodeOptions & options)
  : Node("arm_controller", options),
    feedback_rate_(20.0),
    feedback_frame_("base_link"),
    streaming_(false),
    estop_active_(false),
    simulate_arrival_(true)
{
  declareParams();
  setupActionServer();
  setupGetPosition();
  setupEmergencyStop();

  RCLCPP_INFO(get_logger(), "arm_controller ready (set_pos action / get_pos streaming)");
}

DeltaArmControllerNode::~DeltaArmControllerNode()
{
  feedback_timer_->cancel();
}

void DeltaArmControllerNode::declareParams()
{
  feedback_rate_ = declare_parameter<double>("feedback_rate", 20.0);
  feedback_frame_ = declare_parameter<std::string>("feedback_frame", "base_link");

  // Mechanism geometry (mm) - shared with the visualisers via arm_params.yaml.
  const double base_radius =
    declare_parameter<double>("geometry.base_radius", 150.0);
  const double platform_radius =
    declare_parameter<double>("geometry.platform_radius", 60.0);
  const double upper_arm_len =
    declare_parameter<double>("geometry.upper_arm_len", 160.0);
  const double lower_arm_len =
    declare_parameter<double>("geometry.lower_arm_len", 320.0);

  // Visual-only servo-linkage parameters (the controller does not use them;
  // declared so the shared geometry block loads without "not declared"
  // warnings). Mechanism per arm: servo shaft -> upper rod (single bar)
  // -> lower rod (single bar) -> upper arm at a fixed distance from the base
  // joint (arm_attach_dist). The lower arm (elbow->platform) is drawn as 2
  // parallel bars separated by rod_spread.
  declare_parameter<double>("geometry.servo_radius", 150.0);
  declare_parameter<double>("geometry.servo_z", -25.0);
  declare_parameter<double>("geometry.upper_rod_len", 60.0);
  declare_parameter<double>("geometry.arm_attach_dist", 30.0);
  declare_parameter<double>("geometry.rod_spread", 8.0);

  simulate_arrival_ = declare_parameter<bool>("sim.simulate_arrival", true);
  const auto initial = declare_parameter<std::vector<double>>("sim.initial_pos", std::vector<double>{0.0, 0.0, -0.30});

  std::vector<DeltaArm::ArmMechConfig> configs;
  constexpr double kTwoPiOver3 = 2.0943951023931953;  // 120 deg
  for (int leg = 0; leg < 3; ++leg) {
    DeltaArm::ArmMechConfig c;
    c.base_radius = static_cast<float>(base_radius);
    c.platform_radius = static_cast<float>(platform_radius);
    c.upper_arm_len = static_cast<float>(upper_arm_len);
    c.lower_arm_len = static_cast<float>(lower_arm_len);
    c.plane_angle = static_cast<float>(leg * kTwoPiOver3);
    c.home_offset = 0.0f;
    configs.push_back(c);
  }
  arm_.set_geometry(configs);

  // Start the arm at a valid (IK-reachable) posture instead of the degenerate
  // (0,0,0). The commanded pose becomes the streamed pose in sim mode.
  if (initial.size() == 3) {
    const float ix = static_cast<float>(initial[0]);
    const float iy = static_cast<float>(initial[1]);
    const float iz = static_cast<float>(initial[2]);
    arm_.set_tar_pos(ix, iy, iz);
    if (simulate_arrival_) arm_.apply();
    float init_deg[3] = {0.0f, 0.0f, 0.0f};
    arm_.get_motor_current(init_deg);
    RCLCPP_INFO(get_logger(),
                "initial pose -> (%.3f, %.3f, %.3f) m, motors %.2f / %.2f / %.2f deg",
                ix, iy, iz, init_deg[0], init_deg[1], init_deg[2]);
  }

  RCLCPP_INFO(get_logger(),
              "geometry: base_radius=%.1f platform_radius=%.1f "
              "upper_arm_len=%.1f lower_arm_len=%.1f",
              base_radius, platform_radius, upper_arm_len, lower_arm_len);
}

void DeltaArmControllerNode::setupActionServer()
{
  motor_pub_ = create_publisher<arm::msg::MotorTargets>("arm/motor_targets", 10);

  action_server_ = rclcpp_action::create_server<SetPosition>(
    this,
    "arm/set_pos",
    std::bind(&DeltaArmControllerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DeltaArmControllerNode::handleCancel, this, std::placeholders::_1),
    std::bind(&DeltaArmControllerNode::executeAction, this, std::placeholders::_1));
}

void DeltaArmControllerNode::setupGetPosition()
{
  pos_pub_ = create_publisher<arm::msg::ArmPosition>("arm/pos", 10);
  // Separate debug topic for the joints (arm endpoint lives on arm/pos).
  joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("arm/joints", 10);

  toggle_srv_ = create_service<arm::srv::TogglePositionStream>(
    "arm/get_pos",
    [this](const std::shared_ptr<arm::srv::TogglePositionStream::Request> req,
           const std::shared_ptr<arm::srv::TogglePositionStream::Response> resp) {
      streaming_ = req->enable;
      resp->success = true;
      resp->streaming = streaming_;
      RCLCPP_INFO(get_logger(), "get_pos streaming %s",
                  streaming_ ? "ENABLED" : "disabled");
    });

  const auto period = std::chrono::milliseconds(
    static_cast<int64_t>(1000.0 / (feedback_rate_ > 0.0 ? feedback_rate_ : 1.0)));
  feedback_timer_ = create_wall_timer(period, [this]() { publishPosition(); });
}

void DeltaArmControllerNode::setupEmergencyStop()
{
  estop_srv_ = create_service<arm::srv::EmergencyStop>(
    "arm/emergency_stop",
    [this](const std::shared_ptr<arm::srv::EmergencyStop::Request> req,
           const std::shared_ptr<arm::srv::EmergencyStop::Response> resp) {
      if (req->activate) {
        // 1) Cross out any in-flight set_pos goal.
        if (current_goal_handle_ && current_goal_handle_->is_active()) {
          auto res = std::make_shared<SetPosition::Result>();
          res->succeeded = false;
          res->message = "aborted by emergency stop";
          current_goal_handle_->abort(res);
          current_goal_handle_.reset();
        }

        // 2) Zero the arm state and republish the (zeroed) motor output.
        arm_.stop();
        const float zero[3] = {0.0f, 0.0f, 0.0f};
        publishTargets(zero);

        // 3) Latch: reject new goals until released; also pause streaming.
        estop_active_ = true;
        streaming_ = false;

        resp->success = true;
        resp->emergency_active = true;
        RCLCPP_WARN(get_logger(), "EMERGENCY STOP - motors zeroed, actions halted");
      } else {
        estop_active_ = false;
        resp->success = true;
        resp->emergency_active = false;
        RCLCPP_INFO(get_logger(), "emergency stop released");
      }
    });
}

// ---------------------------------------------------------------------------
// set_pos action server
// ---------------------------------------------------------------------------
rclcpp_action::GoalResponse DeltaArmControllerNode::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const SetPosition::Goal> /*goal*/)
{
  if (estop_active_) {
    RCLCPP_WARN(get_logger(), "set_pos goal REJECTED - emergency stop active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "set_pos goal received - accepting");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DeltaArmControllerNode::handleCancel(
  const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
  RCLCPP_INFO(get_logger(), "set_pos goal cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DeltaArmControllerNode::executeAction(const std::shared_ptr<GoalHandle> goal_handle)
{
  current_goal_handle_ = goal_handle;
  const auto goal = *goal_handle->get_goal();

  const float x = static_cast<float>(goal.target.linear.x);
  const float y = static_cast<float>(goal.target.linear.y);
  const float z = static_cast<float>(goal.target.linear.z);

  RCLCPP_INFO(get_logger(), "set_pos -> (%.3f, %.3f, %.3f)", x, y, z);

  // Stage the task-space target and run the (two-stage) IK. The result is
  // published to the driver on arm/motor_targets.
  arm_.set_tar_pos(x, y, z);

  float angles[3] = {0.0f, 0.0f, 0.0f};
  arm_.get_motor_targets(angles);
  publishTargets(angles);

  // Sim mode: there is no motor driver to close the loop, so promote the
  // commanded target to the current pose immediately. The arm/pos stream then
  // reflects the requested pose and the visualiser can draw the motion.
  if (simulate_arrival_) arm_.apply();

  // Feedback: current end-effector estimate (skeleton: streamed estimate).
  auto feedback = std::make_shared<SetPosition::Feedback>();
  const DeltaArm::Vec3 cur = arm_.get_cur_pos();
  feedback->current.x = cur.x;
  feedback->current.y = cur.y;
  feedback->current.z = cur.z;
  goal_handle->publish_feedback(feedback);

  // TODO(user): track arrival (compare driver feedback against target, then
  // finalize only once within tolerance). For now we accept immediately.
  auto result = std::make_shared<SetPosition::Result>();
  result->succeeded = true;
  result->message = "target accepted and forwarded to driver";
  goal_handle->succeed(result);

  if (current_goal_handle_ == goal_handle) current_goal_handle_.reset();
}

// ---------------------------------------------------------------------------
// get_pos / stream helpers
// ---------------------------------------------------------------------------
void DeltaArmControllerNode::publishTargets(const float * angles)
{
  auto msg = std::make_shared<arm::msg::MotorTargets>();
  msg->header.stamp = now();
  msg->header.frame_id = feedback_frame_;
  msg->angles[0] = angles[0];
  msg->angles[1] = angles[1];
  msg->angles[2] = angles[2];
  motor_pub_->publish(*msg);
}

void DeltaArmControllerNode::publishPosition()
{
  if (!streaming_) return;

  auto msg = std::make_shared<arm::msg::ArmPosition>();
  msg->header.stamp = now();
  msg->header.frame_id = feedback_frame_;

  const DeltaArm::Vec3 cur = arm_.get_cur_pos();
  msg->position.x = cur.x;
  msg->position.y = cur.y;
  msg->position.z = cur.z;

  float cur_angles[3] = {0.0f, 0.0f, 0.0f};
  float tar_angles[3] = {0.0f, 0.0f, 0.0f};
  arm_.get_motor_current(cur_angles);
  arm_.get_motor_targets(tar_angles);
  for (int i = 0; i < 3; ++i) {
    msg->motor_angles_current[i] = cur_angles[i];
    msg->motor_angles_target[i] = tar_angles[i];
  }

  pos_pub_->publish(*msg);

  // Debug-only joint state topic: current joint angles (degrees rad converted).
  auto js = std::make_shared<sensor_msgs::msg::JointState>();
  js->header.stamp = now();
  js->header.frame_id = feedback_frame_;
  js->name = {"joint_0", "joint_1", "joint_2"};
  js->position = {cur_angles[0], cur_angles[1], cur_angles[2]};
  joints_pub_->publish(*js);
}

}  // namespace DeltaArmRos