// Delta-arm Gazebo (Classic) model plugin.
//
// Bridges the arm controller into the simulated 3DOF delta arm:
//   * subscribes "arm/motor_targets" (arm/msg/MotorTargets) - commanded joint
//     angles in DEGREES (single publication per accepted set_pos goal)
//   * servoes the three shoulder joints with a PD torque control on the upper
//     arm links. The shoulders sit inside the delta's closed loop, where ODE
//     rejects kinematic SetPosition and barely honours SetVelocity; a torque
//     applied to the upper-arm body composes correctly with the loop
//     constraints and drives the mechanism.
//
// Configured from the model's <plugin> SDF:
//   <shoulder_0>joint_name</shoulder_0> ... optional joint name overrides
//   <kp>25.0</kp>           proportional torque gain (Nm per rad of error)
//   <ki>8.0</ki>            integral torque gain (Nm per rad*s of error)
//   <kd>1.2</kd>            derivative torque gain (Nm per rad/s)
//   <tau_max>15.0</tau_max>   torque clamp (Nm)

#include <algorithm>
#include <cmath>
#include <exception>
#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <arm/msg/motor_targets.hpp>

#include <ignition/math/Vector3.hh>

namespace arm_gazebo
{

class DeltaArmGazeboPlugin : public gazebo::ModelPlugin
{
public:
  DeltaArmGazeboPlugin() = default;
  ~DeltaArmGazeboPlugin() override = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    if (!model_) {
      gzerr << "DeltaArmGazeboPlugin: no model\n";
      return;
    }

    // Joint + link names: optional SDF overrides, otherwise "<id>".
    const std::string default_names[3] = {"shoulder_0", "shoulder_1", "shoulder_2"};
    try {
      for (int i = 0; i < 3; ++i) {
        std::string name = default_names[i];
        if (sdf->HasElement("shoulder_" + std::to_string(i))) {
          name = sdf->Get<std::string>("shoulder_" + std::to_string(i));
        }
        shoulder_[i] = model_->GetJoint(name);
        if (!shoulder_[i]) {
          gzerr << "DeltaArmGazeboPlugin: missing joint [" << name << "]\n";
          return;
        }
        upper_arm_[i] = shoulder_[i]->GetChild();
        if (!upper_arm_[i]) {
          gzerr << "DeltaArmGazeboPlugin: no child link for joint [" << name << "]\n";
          return;
        }
      }
    } catch (const std::exception & e) {
      gzerr << "DeltaArmGazeboPlugin: failed to resolve joints: " << e.what() << "\n";
      return;
    }

    kp_ = sdf->Get<double>("kp", kp_).first;
    ki_ = sdf->Get<double>("ki", ki_).first;
    kd_ = sdf->Get<double>("kd", kd_).first;
    tau_max_ = sdf->Get<double>("tau_max", tau_max_).first;

    std::string topic = "arm/motor_targets";
    try {
      if (sdf->HasElement("enable_topic")) {
        topic = sdf->Get<std::string>("enable_topic");
      }

      node_ = gazebo_ros::Node::Get(sdf);
      sub_ = node_->create_subscription<arm::msg::MotorTargets>(
        topic, rclcpp::SensorDataQoS(),
        [this](const arm::msg::MotorTargets::ConstSharedPtr msg) { onMotorTargets(msg); });

      update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&DeltaArmGazeboPlugin::onUpdate, this));

      gzmsg << "DeltaArmGazeboPlugin loaded: joints "
            << shoulder_[0]->GetName() << ", " << shoulder_[1]->GetName() << ", "
            << shoulder_[2]->GetName()
            << " | subscribed to [" << topic << "]\n";
    } catch (const std::exception & e) {
      gzerr << "DeltaArmGazeboPlugin: load error: " << e.what() << "\n";
    }
  }

private:
  void onMotorTargets(const arm::msg::MotorTargets::ConstSharedPtr msg)
  {
    const int n = std::min<int>(3, static_cast<int>(msg->angles.size()));
    for (int i = 0; i < n; ++i) {
      target_[i] = msg->angles[i] * kDegToRad;
    }
    gzmsg << "DeltaArmGazeboPlugin: motor targets (deg) "
          << msg->angles[0] << ", " << msg->angles[1] << ", " << msg->angles[2] << "\n";
  }

  void onUpdate()
  {
    if (!model_) {
      return;
    }

    const double t = model_->GetWorld()->SimTime().Double();
    const double dt = std::min(t - last_time_, 0.05);
    last_time_ = t;
    if (dt <= 0.0) {
      return;
    }

    // PD+I torque around each shoulder axis (the upper arm's local +X). The
    // integral removes the steady-state droop against gravity / loop forces.
    for (int i = 0; i < 3; ++i) {
      const double cur = shoulder_[i]->Position(0);
      const double vel = shoulder_[i]->GetVelocity(0);
      const double err = target_[i] - cur;
      integral_[i] += err * dt;
      integral_[i] = std::clamp(integral_[i], -i_max_, i_max_);
      const double tau = std::clamp(
        kp_ * err + ki_ * integral_[i] - kd_ * vel, -tau_max_, tau_max_);
      upper_arm_[i]->AddRelativeTorque(ignition::math::Vector3d(tau, 0.0, 0.0));
    }
  }

  static constexpr double kDegToRad = 0.017453292519943295;

  gazebo::physics::ModelPtr model_;
  gazebo::physics::JointPtr shoulder_[3];
  gazebo::physics::LinkPtr upper_arm_[3];
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<arm::msg::MotorTargets>::SharedPtr sub_;

  double target_[3] = {0.0, 0.0, 0.0};
  double integral_[3] = {0.0, 0.0, 0.0};
  double last_time_ = -1.0;
  double kp_ = 25.0;
  double ki_ = 8.0;
  double kd_ = 1.2;
  double tau_max_ = 15.0;
  double i_max_ = 0.5;
};

GZ_REGISTER_MODEL_PLUGIN(DeltaArmGazeboPlugin)

}  // namespace arm_gazebo