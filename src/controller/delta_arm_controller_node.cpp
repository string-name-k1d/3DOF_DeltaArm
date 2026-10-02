#include "arm/delta_arm_controller.hpp"

#include <array>
#include <chrono>
#include <functional>
#include <vector>

namespace DeltaArmRos {

DeltaArmControllerNode::DeltaArmControllerNode(const rclcpp::NodeOptions& options)
    : Node("arm_controller", options), feedback_rate_(20.0), feedback_frame_("base_link"), streaming_(false),
      simulate_arrival_(true), enable_motion_(true) {
    declareParams();
    setupActionServer();
    setupGetPosition();
    setupEmergencyStop();

    RCLCPP_INFO(get_logger(), "arm_controller ready (set_pos action / get_pos streaming), state=%s",
                ArmStateControl::toString(state_control_.getState()));
}

DeltaArmControllerNode::~DeltaArmControllerNode() { feedback_timer_->cancel(); }

void DeltaArmControllerNode::declareParams() {
    feedback_rate_ = declare_parameter<double>("feedback_rate", 20.0);
    feedback_frame_ = declare_parameter<std::string>("feedback_frame", "base_link");

    // Mechanism geometry (mm) - shared with the visualisers via arm_params.yaml.
    const double base_radius = declare_parameter<double>("geometry.base_radius", 100.0);
    const double platform_radius = declare_parameter<double>("geometry.platform_radius", 32.5);
    const double upper_arm_len = declare_parameter<double>("geometry.upper_arm_len", 120.0);
    const double lower_arm_len = declare_parameter<double>("geometry.lower_arm_len", 240.0);

    // 4-bar servo-linkage geometry (see ArmMechConfig): the servo drives the
    // upper arm through a horn + rigid rod, mapped by the exact circle-circle
    // solve in ik_stage2. c = hypot(arm_attach_dist, arm_attach_offset); the
    // ground link d runs servo -> shoulder.
    const double servo_radius = declare_parameter<double>("geometry.servo_radius", 57.65);
    const double servo_z = declare_parameter<double>("geometry.servo_z", -22.5);
    const double upper_rod_len = declare_parameter<double>("geometry.upper_rod_len", 60.0);
    const double servo_rod_len = declare_parameter<double>("geometry.servo_rod_len", 35.0);
    const double arm_attach_dist = declare_parameter<double>("geometry.arm_attach_dist", 68.5);
    const double arm_attach_offset = declare_parameter<double>("geometry.arm_attach_offset", 20.5);
    declare_parameter<double>("geometry.rod_spread", 8.0);

    // Travel limits. The servo's own mechanical travel (angle_min/angle_max, in
    // degrees) is the same pair the motor driver clamps against; it bounds the
    // IK so a pose is never commanded that the servo could not hold. The
    // arm-angle window is an additional practical bound on the limb itself: a
    // negative arm angle is mechanically possible on some assemblies but rarely
    // useful, so it is excluded by default and can be re-enabled by setting
    // geometry.arm_angle_min below zero.
    const auto angle_min = declare_parameter<std::vector<double>>("geometry.angle_min", std::vector<double>{0.0, 0.0, 0.0});
    const auto angle_max =
        declare_parameter<std::vector<double>>("geometry.angle_max", std::vector<double>{145.0, 145.0, 145.0});
    const double arm_angle_min = declare_parameter<double>("geometry.arm_angle_min", 0.0);
    const double arm_angle_max = declare_parameter<double>("geometry.arm_angle_max", 180.0);
    // Diagnostic escape hatch: when true the IK stops refusing out-of-envelope
    // targets and resolves them to the nearest closeable arm angle. Default false
    // keeps the reachability guard ON. See ArmMechConfig::bypass_reachability.
    const bool bypass_reachability = declare_parameter<bool>("geometry.bypass_reachability", false);
    constexpr double kDegToRad = 0.017453292519943295;

    simulate_arrival_ = declare_parameter<bool>("sim.simulate_arrival", true);
    enable_motion_ = declare_parameter<bool>("enable_motion", true);
    const auto initial =
        declare_parameter<std::vector<double>>("sim.initial_pos", std::vector<double>{0.0, 0.0, -0.30});

    // Control-augmentation pipeline (see arm/control_augmenter.hpp): per-limb
    // offset -> feedforward -> slew clamp, applied to every MotorTargets message
    // published on arm/motor_targets. Identity by default (offsets zero, FF off,
    // no slew clamp), so enabling the node never changes the robot on its own.
    const auto offset_deg = declare_parameter<std::vector<double>>("offset_deg", std::vector<double>{0.0, 0.0, 0.0});
    std::array<double, 3> offset{0.0, 0.0, 0.0};
    for (size_t i = 0; i < 3 && i < offset_deg.size(); ++i) {
        offset[i] = offset_deg[i];
    }
    const bool enable_feedforward = declare_parameter<bool>("enable_feedforward", false);
    const double max_delta_deg = declare_parameter<double>("max_delta_deg", 0.0);
    augmenter_.configure(offset, enable_feedforward, max_delta_deg);
    RCLCPP_INFO(get_logger(), "control augmenter: offsets=[%.2f, %.2f, %.2f] deg, feedforward=%s, slew=%s", offset[0],
                offset[1], offset[2], enable_feedforward ? "on" : "off", max_delta_deg > 0.0 ? "on" : "off");

    // Control-state machine (see arm/arm_state_control.hpp): the safe home
    // posture used when the arm returns home / after an estop release. The node
    // boots in kHome, so goals are accepted until an estop latches kZeroEmergency.
    const auto home_pos = declare_parameter<std::vector<double>>("control.home_pos", std::vector<double>{0.0, 0.0, -0.30});
    std::array<double, 3> home{0.0, 0.0, -0.30};
    for (size_t i = 0; i < 3 && i < home_pos.size(); ++i) {
        home[i] = home_pos[i];
    }
    state_control_.configureHome(home);
    RCLCPP_INFO(get_logger(), "arm control state: %s, home=(%.3f, %.3f, %.3f) m",
                ArmStateControl::toString(state_control_.getState()), home[0], home[1], home[2]);

    std::vector<DeltaArm::ArmMechConfig> configs;
    constexpr double kTwoPiOver3 = 2.0943951023931953; // 120 deg
    for (int leg = 0; leg < 3; ++leg) {
        // The limits are per-limb in the config; the parameter arrays are
        // shorter-tolerant than the YAML suggests, so fall back per index.
        const size_t idx = static_cast<size_t>(leg);
        DeltaArm::ArmMechConfig c{
            .base_radius = static_cast<float>(base_radius),
            .platform_radius = static_cast<float>(platform_radius),
            .upper_arm_len = static_cast<float>(upper_arm_len),
            .lower_arm_len = static_cast<float>(lower_arm_len),
            .servo_radius = static_cast<float>(servo_radius),
            .servo_z = static_cast<float>(servo_z),
            .upper_rod_len = static_cast<float>(upper_rod_len),
            .servo_rod_len = static_cast<float>(servo_rod_len),
            .arm_attach_dist = static_cast<float>(arm_attach_dist),
            .arm_attach_offset = static_cast<float>(arm_attach_offset),
            .motor_angle_min = static_cast<float>(idx < angle_min.size() ? angle_min[idx] : 0.0),
            .motor_angle_max = static_cast<float>(idx < angle_max.size() ? angle_max[idx] : 145.0),
            .arm_angle_min = static_cast<float>(arm_angle_min * kDegToRad),
            .arm_angle_max = static_cast<float>(arm_angle_max * kDegToRad),
            .plane_angle = static_cast<float>(leg * kTwoPiOver3),
            .home_offset = 0.0f,
            .bypass_reachability = bypass_reachability,
        };
        configs.push_back(c);
    }
    arm_.set_geometry(configs);
    RCLCPP_WARN(get_logger(), "geometry.bypass_reachability=%s (reachability guard is %s)", bypass_reachability ? "true" : "false",
                bypass_reachability ? "DISABLED - out-of-envelope targets are clamped, not refused" : "ON");

    // Start the arm at a valid (IK-reachable) posture instead of the degenerate
    // (0,0,0). The commanded pose becomes the streamed pose in sim mode.
    if (initial.size() == 3) {
        const float ix = static_cast<float>(initial[0]);
        const float iy = static_cast<float>(initial[1]);
        const float iz = static_cast<float>(initial[2]);
        const DeltaArm::TargetResult init = arm_.set_tar_pos(ix, iy, iz);

        if (simulate_arrival_ && init.reached)
            arm_.apply();

        float init_deg[3] = {0.0f, 0.0f, 0.0f};
        arm_.get_motor_current(init_deg);
        if (init.reached) {
            RCLCPP_INFO(get_logger(), "initial pose -> (%.3f, %.3f, %.3f) m, motors %.2f / %.2f / %.2f deg", ix, iy, iz,
                        init_deg[0], init_deg[1], init_deg[2]);
        } else {
            RCLCPP_ERROR(get_logger(), "initial pose (%.3f, %.3f, %.3f) m is UNREACHABLE: %s - the arm stays at its "
                                       "start angles. Fix sim.initial_pos or the geometry.",
                         ix, iy, iz, init.reason.c_str());
        }
    }

    RCLCPP_INFO(get_logger(),
                "geometry: base_radius=%.1f platform_radius=%.1f "
                "upper_arm_len=%.1f lower_arm_len=%.1f",
                base_radius, platform_radius, upper_arm_len, lower_arm_len);
    RCLCPP_INFO(get_logger(),
                "linkage: servo(r=%.1f z=%.1f) horn=%.1f rod=%.1f "
                "attach=%.1f offset=%.1f",
                servo_radius, servo_z, upper_rod_len, servo_rod_len, arm_attach_dist, arm_attach_offset);

    // Report the resolved reachable arm-angle band: it is derived, so a
    // surprising workspace is far easier to diagnose from the log than by
    // probing the arm.
    for (int leg = 0; leg < 3; ++leg) {
        float lo_deg = 0.0f;
        float hi_deg = 0.0f;
        if (arm_.get_linkage_band(leg, lo_deg, hi_deg)) {
            RCLCPP_INFO(get_logger(), "reachable arm band, leg %d: [%.2f, %.2f] deg (servo travel [%.1f, %.1f] deg)", leg,
                        lo_deg, hi_deg, angle_min.empty() ? 0.0 : angle_min.front(),
                        angle_max.empty() ? 145.0 : angle_max.front());
        } else {
            RCLCPP_WARN(get_logger(), "reachable arm band, leg %d: EMPTY - the 4-bar cannot close within the servo's "
                                     "travel. Every pose will be rejected.",
                        leg);
        }
    }
}

void DeltaArmControllerNode::setupActionServer() {
    motor_pub_ = create_publisher<arm::msg::MotorTargets>("arm/motor_targets", 10);
    control_state_pub_ = create_publisher<arm::msg::ControlState>("arm/control_state", 10);
    publishControlState();

    if (!enable_motion_) {
        RCLCPP_INFO(get_logger(), "motion control DISABLED (enable_motion=false) - set_pos action not started");
        return;
    }

    action_server_ = rclcpp_action::create_server<SetPosition>(
        this, "arm/set_pos",
        std::bind(&DeltaArmControllerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DeltaArmControllerNode::handleCancel, this, std::placeholders::_1),
        std::bind(&DeltaArmControllerNode::executeAction, this, std::placeholders::_1));
}

void DeltaArmControllerNode::setupGetPosition() {
    pos_pub_ = create_publisher<arm::msg::ArmPosition>("arm/pos", 10);
    // Separate debug topic for the joints (arm endpoint lives on arm/pos).
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("arm/joints", 10);

    toggle_srv_ = create_service<arm::srv::TogglePositionStream>(
        "arm/get_pos", [this](const std::shared_ptr<arm::srv::TogglePositionStream::Request> req,
                              const std::shared_ptr<arm::srv::TogglePositionStream::Response> resp) {
            streaming_ = req->enable;
            resp->success = true;
            resp->streaming = streaming_;
            RCLCPP_INFO(get_logger(), "get_pos streaming %s", streaming_ ? "ENABLED" : "disabled");
        });

    const auto period =
        std::chrono::milliseconds(static_cast<int64_t>(1000.0 / (feedback_rate_ > 0.0 ? feedback_rate_ : 1.0)));
    feedback_timer_ = create_wall_timer(period, [this]() { publishPosition(); });
}

void DeltaArmControllerNode::setupEmergencyStop() {
    estop_srv_ = create_service<arm::srv::EmergencyStop>(
        "arm/emergency_stop", [this](const std::shared_ptr<arm::srv::EmergencyStop::Request> req,
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

                // 2) Latch the zero/emergency state, zero the arm, and
                //    republish the (zeroed) motor output. Reset the augmenter
                //    first so the estop zero is delivered immediately and is
                //    not clamped by the slew limiter.
                state_control_.activateEmergencyStop("emergency stop service");
                augmenter_.reset();
                arm_.stop();
                const float zero[3] = {0.0f, 0.0f, 0.0f};
                publishTargets(zero);
                publishControlState();

                // 3) Latch: reject new goals until released; also pause streaming.
                streaming_ = false;

                resp->success = true;
                resp->emergency_active = true;
                RCLCPP_WARN(get_logger(), "EMERGENCY STOP - motors zeroed, actions halted, state=%s",
                            ArmStateControl::toString(state_control_.getState()));
            } else {
                state_control_.releaseEmergencyStop("emergency stop service");
                publishControlState();
                resp->success = true;
                resp->emergency_active = false;
                RCLCPP_INFO(get_logger(), "emergency stop released, state=%s",
                            ArmStateControl::toString(state_control_.getState()));
            }
        });
}

// ---------------------------------------------------------------------------
// set_pos action server
// ---------------------------------------------------------------------------
rclcpp_action::GoalResponse DeltaArmControllerNode::handleGoal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                               std::shared_ptr<const SetPosition::Goal> /*goal*/) {
    if (!state_control_.acceptsGoals()) {
        RCLCPP_WARN(get_logger(), "set_pos goal REJECTED - state is %s (%s)",
                    ArmStateControl::toString(state_control_.getState()), state_control_.getLastReason().c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    state_control_.onGoalAccepted();
    publishControlState();
    RCLCPP_INFO(get_logger(), "set_pos goal received - accepting (state=%s)",
                ArmStateControl::toString(state_control_.getState()));
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DeltaArmControllerNode::handleCancel(const std::shared_ptr<GoalHandle> /*goal_handle*/) {
    RCLCPP_INFO(get_logger(), "set_pos goal cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DeltaArmControllerNode::executeAction(const std::shared_ptr<GoalHandle> goal_handle) {
    current_goal_handle_ = goal_handle;
    const auto goal = *goal_handle->get_goal();

    const float x = static_cast<float>(goal.target.linear.x);
    const float y = static_cast<float>(goal.target.linear.y);
    const float z = static_cast<float>(goal.target.linear.z);

    RCLCPP_INFO(get_logger(), "set_pos -> (%.3f, %.3f, %.3f)", x, y, z);

    // Stage the task-space target and run the (two-stage) IK. The result is
    // published to the driver on arm/motor_targets.
    const DeltaArm::TargetResult reach = arm_.set_tar_pos(x, y, z);
    if (!reach.reached) {
        // Reporting success here used to make an unreachable goal look exactly
        // like a dead motor: the arm simply did not move. Abort instead, and
        // name the target so the caller can see WHICH pose was refused.
        RCLCPP_WARN(get_logger(), "set_pos REJECTED (%.3f, %.3f, %.3f) m: %s", x, y, z, reach.reason.c_str());
        auto result = std::make_shared<SetPosition::Result>();
        result->succeeded = false;
        result->message = std::string("target outside the reachable envelope: ") + reach.reason;
        goal_handle->abort(result);

        if (current_goal_handle_ == goal_handle)
            current_goal_handle_.reset();
        return;
    }

    float angles[3] = {0.0f, 0.0f, 0.0f};
    arm_.get_motor_targets(angles);
    publishTargets(angles);

    // Sim mode: there is no motor driver to close the loop, so promote the
    // commanded target to the current pose immediately. The arm/pos stream then
    // reflects the requested pose and the visualiser can draw the motion.
    if (simulate_arrival_)
        arm_.apply();

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

    if (current_goal_handle_ == goal_handle)
        current_goal_handle_.reset();
}

// ---------------------------------------------------------------------------
// get_pos / stream helpers
// ---------------------------------------------------------------------------
void DeltaArmControllerNode::publishTargets(const float* angles) {
    auto msg = std::make_shared<arm::msg::MotorTargets>();
    msg->header.stamp = now();
    msg->header.frame_id = feedback_frame_;
    msg->angles[0] = angles[0];
    msg->angles[1] = angles[1];
    msg->angles[2] = angles[2];
    motor_pub_->publish(augmenter_.augment(*msg));
}

void DeltaArmControllerNode::publishControlState() {
    if (!control_state_pub_) {
        return;
    }

    auto msg = std::make_shared<arm::msg::ControlState>();
    msg->header.stamp = now();
    msg->header.frame_id = feedback_frame_;
    msg->state = ArmStateControl::toString(state_control_.getState());
    msg->emergency_active = state_control_.isEmergencyActive();
    msg->accepts_goals = state_control_.acceptsGoals();
    msg->last_reason = state_control_.getLastReason();

    const auto& home = state_control_.getHomePose();
    msg->home_position[0] = static_cast<float>(home[0]);
    msg->home_position[1] = static_cast<float>(home[1]);
    msg->home_position[2] = static_cast<float>(home[2]);

    const DeltaArm::Vec3 cur = arm_.get_cur_pos();
    msg->current_position[0] = cur.x;
    msg->current_position[1] = cur.y;
    msg->current_position[2] = cur.z;

    control_state_pub_->publish(*msg);
}

void DeltaArmControllerNode::publishPosition() {
    if (!streaming_)
        return;

    auto msg = std::make_shared<arm::msg::ArmPosition>();
    msg->header.stamp = now();
    msg->header.frame_id = feedback_frame_;

    const DeltaArm::Vec3 cur = arm_.get_cur_pos();
    msg->position.x = cur.x;
    msg->position.y = cur.y;
    msg->position.z = cur.z;

    // Commanded target (control intent) so visualisers can mark it directly.
    const DeltaArm::Vec3 tar = arm_.get_tar_pos();
    msg->target_position.x = tar.x;
    msg->target_position.y = tar.y;
    msg->target_position.z = tar.z;

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

} // namespace DeltaArmRos