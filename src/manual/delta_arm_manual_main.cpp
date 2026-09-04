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
 * @brief Standalone manual-control (WASD) driver.
 *
 * Spins the ManualControlNode in a background thread, then reads WASD keys in
 * the foreground and sends set_pos goals relative to the last commanded target:
 *
 *   W / S : move along the axis0 component (default z) by `step`
 *   A / D : move along the axis1 component (default y) by `step`
 *   Q     : quit
 *
 * Current-position changes are printed by the node from the get_pos stream.
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DeltaArmRos::ManualControlNode>(rclcpp::NodeOptions());
  const double step = node->declare_parameter<double>("step", 0.01);
  const char axis0 = node->declare_parameter<std::string>("axis0", "z")[0];
  const char axis1 = node->declare_parameter<std::string>("axis1", "y")[0];

  auto spinner = std::thread([node]() { rclcpp::spin(node); });

  // Give the executor a moment to start before requesting the position stream.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  node->enable_position_streaming();

  std::printf("\nDelta-arm manual control (WASD)\n");
  std::printf("  W/S : +/- %c   A/D : +/- %c   Q : quit\n\n", axis0, axis1);
  std::fflush(stdout);

  // Add `delta` to the component selected by `axis`.
  auto inc_axis = [](double & x, double & y, double & z, char axis, double delta) {
    switch (axis) {
      case 'x': x += delta; break;
      case 'y': y += delta; break;
      default:  z += delta; break;
    }
  };

  double tx = 0.0, ty = 0.0, tz = 0.0;

  bool running = true;
  while (running && rclcpp::ok()) {
    const int key = getch_one();
    switch (key) {
      case 'w': case 'W':
        inc_axis(tx, ty, tz, axis0, +step); break;
      case 's': case 'S':
        inc_axis(tx, ty, tz, axis0, -step); break;
      case 'a': case 'A':
        inc_axis(tx, ty, tz, axis1, -step); break;
      case 'd': case 'D':
        inc_axis(tx, ty, tz, axis1, +step); break;
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