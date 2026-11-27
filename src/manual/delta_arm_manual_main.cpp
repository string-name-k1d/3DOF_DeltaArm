#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <thread>
#include <utility>

#if defined(_WIN32)
#include <conio.h>
#else
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "rclcpp/rclcpp.hpp"

#include "arm/delta_arm_manual.hpp"

namespace
{

using Vec3 = std::array<double, 3>;

const std::array<std::pair<char, Vec3>, 6> kKeyDirections{{
  {'w', {0.0, 1.0, 0.0}},
  {'s', {0.0, -1.0, 0.0}},
  {'a', {-1.0, 0.0, 0.0}},
  {'d', {1.0, 0.0, 0.0}},
  {'z', {0.0, 0.0, 1.0}},
  {'x', {0.0, 0.0, -1.0}},
}};

class TerminalMode
{
public:
  TerminalMode()
  {
#if defined(_WIN32)
    raw_ = true;
#else
    if (::isatty(STDIN_FILENO) == 0 || ::tcgetattr(STDIN_FILENO, &saved_) != 0) {
      return;
    }
    termios raw = saved_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 1;
    raw.c_cc[VTIME] = 0;
    if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      raw_ = true;
    }
#endif
  }

  ~TerminalMode() { restore(); }

  bool raw() const { return raw_; }

  void restore()
  {
#if !defined(_WIN32)
    if (raw_) {
      ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
      raw_ = false;
    }
#endif
  }

private:
  bool raw_{false};
#if !defined(_WIN32)
  termios saved_{};
#endif
};

int read_key()
{
#if defined(_WIN32)
  return _getch();
#else
  unsigned char c = 0;
  while (rclcpp::ok()) {
    pollfd pfd{STDIN_FILENO, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, 200);
    if (rc < 0) {
      if (errno == EINTR) continue;
      return -1;
    }
    if (rc == 0) continue;

    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    if (n == 1) return c;
    if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
    return -1;
  }
  return -1;
#endif
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DeltaArmRos::ManualControlNode>(rclcpp::NodeOptions());
  const double step = node->declare_parameter<double>("step", 0.01);

  auto spinner = std::thread([node]() { rclcpp::spin(node); });

  TerminalMode terminal;

  std::printf("\nDelta-arm manual control (WASD + Z/X)\n");
  std::printf("  W/S : +/- y   A/D : -/+ x   Z/X : +/- z   Q : quit\n");
  if (terminal.raw()) {
    std::printf("  keys act immediately (single keypress, no Enter)\n");
  } else {
    std::printf("  WARNING: stdin is not an interactive terminal.\n");
    std::printf("           Run this node directly (not via ros2 launch) in an\n");
    std::printf("           interactive terminal; keys may need Enter here.\n");
  }
  std::printf("\n");
  std::fflush(stdout);

  while (rclcpp::ok()) {
    const int key = read_key();
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
