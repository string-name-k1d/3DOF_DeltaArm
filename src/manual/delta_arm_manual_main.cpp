#include <cstdio>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "arm/delta_arm_manual.hpp"

namespace
{

// Single-character keyboard read (blocking). POSIX uses a raw termios so each
// keypress is returned immediately without requiring Enter; Windows falls back
// to _getch().
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
 * Current-position changes are printed by the node from the get_pos stream.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DeltaArmRos::ManualControlNode>(rclcpp::NodeOptions());
  const double step = node->declare_parameter<double>("step", 0.01);

  auto spinner = std::thread([node]() { rclcpp::spin(node); });

  // Give the executor a moment to start before requesting the position stream.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  node->enable_position_streaming();

  std::printf("\nDelta-arm manual control (WASD + Z/X)\n");
  std::printf("  W/S : +/- y   A/D : -/+ x   Z/X : +/- z   Q : quit\n\n");
  std::fflush(stdout);

  double tx = 0.0, ty = 0.0, tz = 0.0;

  bool running = true;
  while (running && rclcpp::ok()) {
    const int key = getch_one();
    switch (key) {
      case 'w': case 'W':
        ty += step; break;
      case 's': case 'S':
        ty -= step; break;
      case 'a': case 'A':
        tx -= step; break;
      case 'd': case 'D':
        tx += step; break;
      case 'z': case 'Z':
        tz += step; break;
      case 'x': case 'X':
        tz -= step; break;
      case 'q': case 'Q':
        running = false;
        continue;
      default:
        continue;
    }
    node->send_target(tx, ty, tz);
  }

  rclcpp::shutdown();
  if (spinner.joinable()) spinner.join();
  std::printf("manual control exited\n");
  return 0;
}