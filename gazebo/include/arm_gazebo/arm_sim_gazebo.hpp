#ifndef arm_gazebo__ARM_SIM_GAZEBO_HPP_
#define arm_gazebo__ARM_SIM_GAZEBO_HPP_

#include <memory>
#include <vector>

#include <glm/glm.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm/delta_arm.hpp"

namespace ArmGazebo
{

/**
 * @brief Gazebo 3-D delta-arm simulator.
 *
 * Stands in for the real motor-driver + mechanism when running the arm in
 * simulation: it consumes the same task-space command the controller produces
 * and publishes the resulting end-effector pose and joint angles. The delta-arm
 * kinematics (DeltaArm::Arm) are re-used from the `arm` package, and the 3D
 * geometry is expressed with GLM (glm::vec3), matching the original optional
 * Gazebo target.
 *
 * Interface (identical to the SFML sim so the controller needs no changes):
 *   * subscribes "arm/target_position" (geometry_msgs/Point) - the target;
 *   * publishes "arm/current_position" (geometry_msgs/Point) - current end-
 *     effector position for feedback/monitoring.
 *
 * In a full deployment this node is the proxy that drives a Gazebo model; the
 * Gazebo world / URDF model live alongside in this package (worlds/, urdf/).
 */
class ArmSimGazeboNode : public rclcpp::Node
{
public:
  explicit ArmSimGazeboNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Latest end-effector position (metres).
  const glm::vec3 & current_position() const { return cur_; }

  /// @brief The three servo angles (degrees), one per motor.
  const std::vector<float> & servo_angles() const { return servo_angles_; }

  /// @brief Advance the simulated motion by one step (dt in seconds).
  void step(double dt);

private:
  void onTarget(const geometry_msgs::msg::Point::SharedPtr msg);

  DeltaArm::Arm arm_;

  glm::vec3 sim_;          // current end-effector (metres)
  glm::vec3 tar_;          // commanded target (metres)
  glm::vec3 cur_;          // last published / estimated position
  float move_speed_;       // Cartesian speed (m/s)
  std::vector<float> servo_angles_;  // degrees, one per motor

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr cur_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr tar_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ArmGazebo

#endif  // arm_gazebo__ARM_SIM_GAZEBO_HPP_
