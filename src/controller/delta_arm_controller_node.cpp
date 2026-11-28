#include "arm/delta_arm_controller.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <thread>
#include <vector>

namespace DeltaArmRos {

DeltaArmControllerNode::DeltaArmControllerNode(const rclcpp::NodeOptions& options)
    : Node("arm_controller", options), feedback_rate_(20.0), feedback_frame_("base_link"),
      reach_tolerance_(0.005), reach_timeout_(10.0), simulate_arrival_(true),
      enable_motion_(true), publish_joints_(false), trajectory_enabled_(true),
      trajectory_rate_(20.0), endpoint_max_speed_(0.08), endpoint_max_accel_(0.2),
      motor_max_speed_(30.0), motor_max_accel_(60.0) {
    action_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    declareParams();
    setupActionServer();
    setupMotorFeedback();
    setupPositionFeedback();

    if (enable_motion_) {
        RCLCPP_INFO(get_logger(), "arm_controller ready (set_pos action / arm/pos feedback)");
    } else {
        RCLCPP_INFO(get_logger(), "arm_controller ready (feedback only - set_pos action disabled)");
    }
    RCLCPP_INFO(get_logger(),
                "motion limits: endpoint %s (%.1f Hz, %.3f m/s, %.3f m/s^2), motors %.1f deg/s, %.1f deg/s^2",
                trajectory_enabled_ ? "trajectory" : "direct", trajectory_rate_, endpoint_max_speed_,
                endpoint_max_accel_, motor_max_speed_, motor_max_accel_);
}

DeltaArmControllerNode::~DeltaArmControllerNode() {
    if (feedback_timer_) {
        feedback_timer_->cancel();
    }
}

void DeltaArmControllerNode::declareParams() {
    feedback_rate_ = declare_parameter<double>("feedback_rate", 20.0);
    feedback_frame_ = declare_parameter<std::string>("feedback_frame", "base_link");
    reach_tolerance_ = declare_parameter<double>("reach_tolerance", 0.005);
    reach_timeout_ = declare_parameter<double>("reach_timeout", 10.0);

    trajectory_enabled_ = declare_parameter<bool>("trajectory.enabled", true);
    trajectory_rate_ = declare_parameter<double>("trajectory.rate", 20.0);
    endpoint_max_speed_ = declare_parameter<double>("trajectory.max_speed", 0.08);
    endpoint_max_accel_ = declare_parameter<double>("trajectory.max_accel", 0.2);
    motor_max_speed_ = declare_parameter<double>("motor_limits.max_speed", 30.0);
    motor_max_accel_ = declare_parameter<double>("motor_limits.max_accel", 60.0);

    const double base_radius = declare_parameter<double>("geometry.base_radius", 100.0);
    const double platform_radius = declare_parameter<double>("geometry.platform_radius", 32.5);
    const double upper_arm_len = declare_parameter<double>("geometry.upper_arm_len", 120.0);
    const double lower_arm_len = declare_parameter<double>("geometry.lower_arm_len", 240.0);


    const double servo_radius = declare_parameter<double>("geometry.servo_radius", 57.65);
    const double servo_z = declare_parameter<double>("geometry.servo_z", -22.5);
    const double upper_rod_len = declare_parameter<double>("geometry.upper_rod_len", 60.0);
    const double servo_rod_len = declare_parameter<double>("geometry.servo_rod_len", 35.0);
    const double arm_attach_dist = declare_parameter<double>("geometry.arm_attach_dist", 68.5);
    const double arm_attach_offset = declare_parameter<double>("geometry.arm_attach_offset", 20.5);
    declare_parameter<double>("geometry.rod_spread", 8.0);


    const auto angle_min = declare_parameter<std::vector<double>>("geometry.angle_min", std::vector<double>{0.0, 0.0, 0.0});
    const auto angle_max =
        declare_parameter<std::vector<double>>("geometry.angle_max", std::vector<double>{145.0, 145.0, 145.0});
    const double arm_angle_min = declare_parameter<double>("geometry.arm_angle_min", 0.0);
    const double arm_angle_max = declare_parameter<double>("geometry.arm_angle_max", 180.0);


    const bool bypass_reachability = declare_parameter<bool>("geometry.bypass_reachability", false);


    const bool raw_motor_clamp = declare_parameter<bool>("geometry.raw_motor_clamp", true);
    constexpr double kDegToRad = 0.017453292519943295;

    simulate_arrival_ = declare_parameter<bool>("sim.simulate_arrival", true);

    enable_motion_ = declare_parameter<bool>("enable_motion", true);
    publish_joints_ = declare_parameter<bool>("publish_joints", false);
    const auto initial =
        declare_parameter<std::vector<double>>("sim.initial_pos", std::vector<double>{0.0, 0.0, -0.30});

    std::vector<DeltaArm::ArmMechConfig> configs;
    constexpr double kTwoPiOver3 = 2.0943951023931953;
    for (int leg = 0; leg < 3; ++leg) {


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
            .raw_motor_clamp = raw_motor_clamp,
        };
        configs.push_back(c);
    }
    arm_.set_geometry(configs);
    if (raw_motor_clamp) {
        RCLCPP_WARN(get_logger(),
                    "geometry.raw_motor_clamp=true (TEMPORARY): IK band constraints are DISABLED; "
                    "motor commands are clamped only to [%.1f, %.1f] deg raw travel",
                    angle_min.empty() ? 0.0 : angle_min.front(),
                    angle_max.empty() ? 145.0 : angle_max.front());
    } else {
        RCLCPP_INFO(get_logger(), "geometry.raw_motor_clamp=false (reachability band is enforced)");
    }
    if (bypass_reachability) {
        RCLCPP_WARN(get_logger(), "geometry.bypass_reachability=true (stage-1 workspace misses are tangent-clamped)");
    }


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

    if (!enable_motion_) {
        RCLCPP_INFO(get_logger(), "motion control DISABLED (enable_motion=false) - set_pos action not started");
        return;
    }

    action_server_ = rclcpp_action::create_server<SetPosition>(
        this, "arm/set_pos",
        std::bind(&DeltaArmControllerNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&DeltaArmControllerNode::handleCancel, this, std::placeholders::_1),
        std::bind(&DeltaArmControllerNode::executeAction, this, std::placeholders::_1),
        rcl_action_server_get_default_options(), action_cb_group_);
}

void DeltaArmControllerNode::setupMotorFeedback() {
    motor_fb_sub_ = create_subscription<arm::msg::MotorFeedback>(
        "arm/MotorFeedback", 10,
        std::bind(&DeltaArmControllerNode::onMotorFeedback, this, std::placeholders::_1));
}

void DeltaArmControllerNode::setupPositionFeedback() {
    pos_pub_ = create_publisher<arm::msg::ArmPosition>("arm/pos", 10);


    if (publish_joints_) {
        joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("arm/joints", 10);
    }

    const auto period =
        std::chrono::milliseconds(static_cast<int64_t>(1000.0 / (feedback_rate_ > 0.0 ? feedback_rate_ : 1.0)));
    feedback_timer_ = create_wall_timer(period, [this]() { publishPosition(); });

    RCLCPP_INFO(get_logger(), "publishing arm/pos%s at %.1f Hz",
                publish_joints_ ? " + arm/joints (2-D debug)" : "", feedback_rate_);
}

rclcpp_action::GoalResponse DeltaArmControllerNode::handleGoal(const rclcpp_action::GoalUUID& uuid,
                                                               std::shared_ptr<const SetPosition::Goal> ) {
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        active_goal_uuid_ = uuid;
    }
    RCLCPP_INFO(get_logger(), "set_pos goal received - accepting");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DeltaArmControllerNode::handleCancel(const std::shared_ptr<GoalHandle> ) {
    RCLCPP_INFO(get_logger(), "set_pos goal cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void DeltaArmControllerNode::executeAction(const std::shared_ptr<GoalHandle> goal_handle) {
    {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (goal_handle->get_goal_id() == active_goal_uuid_) {
            current_goal_handle_ = goal_handle;
        }
    }
    const auto goal = *goal_handle->get_goal();

    const float x = static_cast<float>(goal.target.linear.x);
    const float y = static_cast<float>(goal.target.linear.y);
    const float z = static_cast<float>(goal.target.linear.z);

    auto is_current_goal = [this, &goal_handle]() {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        return goal_handle->get_goal_id() == active_goal_uuid_;
    };
    auto finish = [this, &goal_handle](int status, bool success) {
        auto result = std::make_shared<SetPosition::Result>();
        result->status = status;
        if (success) {
            goal_handle->succeed(result);
        } else {
            goal_handle->abort(result);
        }
        clearGoal(goal_handle);
    };

    RCLCPP_INFO(get_logger(), "set_pos -> (%.3f, %.3f, %.3f)", x, y, z);

    DeltaArm::Vec3 start;
    float start_angles[3] = {0.0f, 0.0f, 0.0f};
    DeltaArm::TargetResult reach;
    {
        std::lock_guard<std::mutex> lock(arm_mutex_);
        start = arm_.get_cur_pos();
        arm_.get_motor_current(start_angles);
        reach = arm_.set_tar_pos(x, y, z);
        motor_limiter_.reset(start_angles);
        if (trajectory_enabled_) {
            const DeltaArm::Vec3 goal_point{x, y, z};
            trajectory_.reset(start, goal_point, endpoint_max_speed_, endpoint_max_accel_);
        }
    }
    if (!reach.reached) {
        RCLCPP_WARN(get_logger(), "set_pos REJECTED (%.3f, %.3f, %.3f) m: %s", x, y, z, reach.reason.c_str());
        finish(SetPosition::Result::STATUS_UNREACHABLE, false);
        return;
    }

    const auto start_time = std::chrono::steady_clock::now();
    const auto deadline = start_time + std::chrono::duration<double>(reach_timeout_);
    const int rate_hz = trajectory_rate_ > 0.0 ? static_cast<int>(trajectory_rate_) : 20;
    const auto tick_period = std::chrono::milliseconds(std::max(1, 1000 / rate_hz));
    auto last_tick = start_time;

    while (rclcpp::ok()) {
        const auto tick = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(tick - last_tick).count();
        last_tick = tick;
        if (!(dt > 0.0)) dt = 1.0 / static_cast<double>(rate_hz);

        if (!is_current_goal()) {
            RCLCPP_INFO(get_logger(), "set_pos (%.3f, %.3f, %.3f) superseded by a newer goal", x, y, z);
            finish(SetPosition::Result::STATUS_POS_NOT_REACHED, false);
            return;
        }

        if (goal_handle->is_canceling()) {
            auto result = std::make_shared<SetPosition::Result>();
            result->status = SetPosition::Result::STATUS_POS_NOT_REACHED;
            goal_handle->canceled(result);
            clearGoal(goal_handle);
            return;
        }

        DeltaArm::Vec3 setpoint{x, y, z};
        bool reachable = true;
        std::string reason;
        float limited[3] = {0.0f, 0.0f, 0.0f};
        DeltaArm::Vec3 cur;
        bool trajectory_done = true;
        {
            std::lock_guard<std::mutex> lock(arm_mutex_);
            if (trajectory_enabled_ && !trajectory_.done()) {
                setpoint = trajectory_.update(dt);
            }
            trajectory_done = !trajectory_enabled_ || trajectory_.done();
            const DeltaArm::TargetResult point = arm_.set_tar_pos(setpoint.x, setpoint.y, setpoint.z);
            reachable = point.reached;
            reason = point.reason;
            if (reachable) {
                float desired[3] = {0.0f, 0.0f, 0.0f};
                arm_.get_motor_targets(desired);
                motor_limiter_.limit(desired, dt, motor_max_speed_, motor_max_accel_, limited);
                arm_.set_motor_targets(limited);
                if (!feedback_seen_.load() && simulate_arrival_) {
                    arm_.apply();
                }
                cur = arm_.get_cur_pos();
            }
        }
        if (!reachable) {
            RCLCPP_WARN(get_logger(), "set_pos REJECTED mid-trajectory (%.3f, %.3f, %.3f) m: %s",
                        setpoint.x, setpoint.y, setpoint.z, reason.c_str());
            finish(SetPosition::Result::STATUS_UNREACHABLE, false);
            return;
        }

        publishTargets(limited);

        auto feedback = std::make_shared<SetPosition::Feedback>();
        feedback->current.x = cur.x;
        feedback->current.y = cur.y;
        feedback->current.z = cur.z;
        goal_handle->publish_feedback(feedback);

        const double dx = static_cast<double>(cur.x) - x;
        const double dy = static_cast<double>(cur.y) - y;
        const double dz = static_cast<double>(cur.z) - z;
        const double error = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (error <= reach_tolerance_ && trajectory_done) {
            finish(SetPosition::Result::STATUS_OK, true);
            return;
        }

        if (std::chrono::steady_clock::now() >= deadline) {
            RCLCPP_WARN(get_logger(),
                        "set_pos NOT REACHED (%.3f, %.3f, %.3f) m: %.1f mm from target "
                        "(tolerance %.1f mm, timeout %.1f s)",
                        x, y, z, error * 1000.0, reach_tolerance_ * 1000.0, reach_timeout_);
            finish(SetPosition::Result::STATUS_POS_NOT_REACHED, false);
            return;
        }

        std::this_thread::sleep_until(tick + tick_period);
    }
}

void DeltaArmControllerNode::clearGoal(const std::shared_ptr<GoalHandle>& goal_handle) {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (current_goal_handle_ == goal_handle)
        current_goal_handle_.reset();
}

void DeltaArmControllerNode::onMotorFeedback(const arm::msg::MotorFeedback::SharedPtr msg) {
    feedback_seen_.store(true);

    float angles[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
        const float angle = msg->servo_angle_current[i];
        if (msg->servo_online[i] != 1 || angle < 0.0f || !std::isfinite(angle))
            return;
        angles[i] = angle;
    }

    std::lock_guard<std::mutex> lock(arm_mutex_);
    arm_.set_motor_feedback(angles);
}

void DeltaArmControllerNode::publishTargets(const float* angles) {
    auto msg = std::make_shared<arm::msg::MotorTargets>();
    msg->header.stamp = now();
    msg->header.frame_id = feedback_frame_;
    msg->angles[0] = angles[0];
    msg->angles[1] = angles[1];
    msg->angles[2] = angles[2];
    motor_pub_->publish(*msg);
}

void DeltaArmControllerNode::publishPosition() {
    auto msg = std::make_shared<arm::msg::ArmPosition>();
    msg->header.stamp = now();
    msg->header.frame_id = feedback_frame_;

    DeltaArm::Vec3 cur;
    DeltaArm::Vec3 tar;
    float cur_angles[3] = {0.0f, 0.0f, 0.0f};
    float tar_angles[3] = {0.0f, 0.0f, 0.0f};
    {
        std::lock_guard<std::mutex> lock(arm_mutex_);
        cur = arm_.get_cur_pos();
        tar = arm_.get_tar_pos();
        arm_.get_motor_current(cur_angles);
        arm_.get_motor_targets(tar_angles);
    }

    msg->position.x = cur.x;
    msg->position.y = cur.y;
    msg->position.z = cur.z;


    msg->target_position.x = tar.x;
    msg->target_position.y = tar.y;
    msg->target_position.z = tar.z;

    for (int i = 0; i < 3; ++i) {
        msg->motor_angles_current[i] = cur_angles[i];
        msg->motor_angles_target[i] = tar_angles[i];
    }

    pos_pub_->publish(*msg);


    if (joints_pub_) {
        auto js = std::make_shared<sensor_msgs::msg::JointState>();
        js->header.stamp = now();
        js->header.frame_id = feedback_frame_;
        js->name = {"joint_0", "joint_1", "joint_2"};
        js->position = {cur_angles[0], cur_angles[1], cur_angles[2]};
        joints_pub_->publish(*js);
    }
}

}
