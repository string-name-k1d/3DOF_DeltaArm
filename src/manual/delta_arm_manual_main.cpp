#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "arm/delta_arm_manual.hpp"

namespace
{

using Vec3 = std::array<double, 3>;

// Unit task-space displacement for each keyboard direction.
const std::array<std::pair<char, Vec3>, 6> kKeyDirections{{
  {'w', {0.0, 1.0, 0.0}},
  {'s', {0.0, -1.0, 0.0}},
  {'a', {-1.0, 0.0, 0.0}},
  {'d', {1.0, 0.0, 0.0}},
  {'z', {0.0, 0.0, 1.0}},
  {'x', {0.0, 0.0, -1.0}},
}};

// Single-character keyboard read (blocking). POSIX uses stdio; Windows falls
// back to _getch().
int getch_one()
{
#if defined(_WIN32)
  return _getch();
#else
  int c = std::getchar();
  return c == EOF ? -1 : c;
#endif
}

}  // namespace

/**
 * @brief Standalone manual-control (WASD + Z/X) driver.
 *
 * Spins the ManualControlNode in a background thread, then reads keys in the
 * foreground and sends set_pos goals relative to the last commanded target:
 *
 *   W / S : target y +/-  (horizontal plane, forward / back)
 *   A / D : target x -/+  (horizontal plane, left / right)
 *   Z / X : target z +/-  (vertical, up / down)
 *   Q     : quit
 *
 * Current-position changes are printed by the node from the arm/pos topic.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DeltaArmRos::ManualControlNode>(rclcpp::NodeOptions());
  const double step = node->declare_parameter<double>("step", 0.01);

  auto spinner = std::thread([node]() { rclcpp::spin(node); });

  std::printf("\nDelta-arm manual control (WASD + Z/X)\n");
  std::printf("  W/S : +/- y   A/D : -/+ x   Z/X : +/- z   Q : quit\n\n");
  std::fflush(stdout);

  while (rclcpp::ok()) {
    const int key = getch_one();
    if (key == -1) break;

    const char lower = static_cast<char>(std::tolower(static_cast<unsigned char>(key)));
    if (lower == 'q') break;

    const auto it = std::find_if(
      kKeyDirections.begin(), kKeyDirections.end(),
      [lower](const auto & entry) { return entry.first == lower; });
    if (it == kKeyDirections.end()) continue;

    const Vec3 & direction = it->second;
    node->shift_target({direction[0] * step, direction[1] * step, direction[2] * step});
  }

  rclcpp::shutdown();
  if (spinner.joinable()) spinner.join();
  std::printf("manual control exited\n");
  return 0;
}
