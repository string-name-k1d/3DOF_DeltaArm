#ifndef arm__ARM_SIM_SFML_HPP_
#define arm__ARM_SIM_SFML_HPP_

#include <memory>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm/delta_arm.hpp"

namespace DeltaArmSim
{

/**
 * @brief 2-D graphical (SFML) delta-arm simulator.
 *
 * Runs the same inverse kinematics as the real arm (DeltaArm::Arm), drives a
 * smooth Cartesian trajectory toward the commanded target, and visualises the
 * mechanism with SFML. This node deliberately uses the SAME interface as the
 * hardware/controller so the arm can be driven identically in sim and reality:
 *
 *  * subscribes "arm/target_position" (geometry_msgs/Point) - the target;
 *  * publishes "arm/current_position" (geometry_msgs/Point) - the current
 *    end-effector position for feedback/monitoring.
 *
 * The full 3-DOF (x, y, z) arm is handled by the arm_ geometry; the SFML view
 * shows a side projection and the three limbs.
 */
class ArmSimSFMLNode : public rclcpp::Node
{
public:
  explicit ArmSimSFMLNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Latest end-effector position on screen (world units, pixels).
  float cur_x() const { return cur_x_; }
  float cur_y() const { return cur_y_; }

  /// @brief The three servo angles (degrees) for a limb-based rendering.
  const std::vector<float> & servo_angles() const { return servo_angles_; }

  /// @brief Advance the simulated motion by one step (called each frame).
  void step(double dt);

private:
  void onTarget(const geometry_msgs::msg::Point::SharedPtr msg);

  DeltaArm::Arm arm_;

  float sim_x_, sim_y_, sim_z_;      // current end-effector (metres)
  float tar_x_, tar_y_, tar_z_;      // commanded target (metres)
  float move_speed_;                 // Cartesian speed (m/s)
  std::vector<float> servo_angles_;  // degrees, one per motor

  float cur_x_, cur_y_;              // screen-space current position

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr cur_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr tar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace DeltaArmSim

#endif  // arm__ARM_SIM_SFML_HPP_
