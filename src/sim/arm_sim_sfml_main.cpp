

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <SFML/Graphics.hpp>

#include "arm/arm_sim_sfml.hpp"

#if defined(ARM_SIM_HAVE_X11)
#include <X11/Xlib.h>
#undef None
#endif

static constexpr float kSin120 = 0.8660254037844386f;
static constexpr float kCos120 = -0.5f;
static constexpr float kPi     = 3.141592653589793f;
static constexpr float kTwoPi  = 6.283185307179586f;
static constexpr float kTwoPiOver3 = 2.0943951023931953f;
static constexpr float kRadToDeg   = 57.29577951308232f;

static constexpr float kScale = 1.25f;

static constexpr float kScaleSide = 0.80f;

struct Geom
{
  float base_radius = 100.0f;
  float platform_radius = 32.5f;
  float upper_arm_len = 120.0f;
  float lower_arm_len = 240.0f;

  float t = 0.0f;


  float servo_radius = 57.65f;
  float servo_z = -22.5f;

  float upper_rod_len = 60.0f;

  float servo_rod_len = 35.0f;

  float arm_attach_dist = 68.5f;


  float arm_attach_offset = 20.5f;

  float rod_spread = 8.0f;


  sf::Vector3f motors[3];
  sf::Vector3f plat_off[3];
  sf::Vector3f servos[3];
};

static double geomParam(rclcpp::Node & n, const char * name, double fallback)
{
  if (n.has_parameter(name)) return n.get_parameter(name).as_double();
  return n.declare_parameter<double>(name, fallback);
}

static Geom loadGeometry(rclcpp::Node & n)
{
  Geom g;
  g.base_radius = static_cast<float>(geomParam(n, "geometry.base_radius", 100.0));
  g.platform_radius = static_cast<float>(geomParam(n, "geometry.platform_radius", 32.5));
  g.upper_arm_len = static_cast<float>(geomParam(n, "geometry.upper_arm_len", 120.0));
  g.lower_arm_len = static_cast<float>(geomParam(n, "geometry.lower_arm_len", 240.0));


  g.t = g.base_radius;


  g.motors[0] = sf::Vector3f(0.0f, -g.t, 0.0f);
  g.motors[1] = sf::Vector3f( g.t * kSin120,  g.t * 0.5f, 0.0f);
  g.motors[2] = sf::Vector3f(-g.t * kSin120,  g.t * 0.5f, 0.0f);


  g.servo_radius     = static_cast<float>(geomParam(n, "geometry.servo_radius", 57.65));
  g.servo_z          = static_cast<float>(geomParam(n, "geometry.servo_z", -22.5));
  g.upper_rod_len    = static_cast<float>(geomParam(n, "geometry.upper_rod_len", 60.0));
  g.servo_rod_len    = static_cast<float>(geomParam(n, "geometry.servo_rod_len", 35.0));
  g.arm_attach_dist  = static_cast<float>(geomParam(n, "geometry.arm_attach_dist", 68.5));
  g.arm_attach_offset = static_cast<float>(geomParam(n, "geometry.arm_attach_offset", 20.5));
  g.rod_spread       = static_cast<float>(geomParam(n, "geometry.rod_spread", 8.0));
  g.servos[0] = sf::Vector3f(0.0f, -g.servo_radius, g.servo_z);
  g.servos[1] = sf::Vector3f( g.servo_radius * kSin120,  g.servo_radius * 0.5f, g.servo_z);
  g.servos[2] = sf::Vector3f(-g.servo_radius * kSin120,  g.servo_radius * 0.5f, g.servo_z);

  g.plat_off[0] = sf::Vector3f(0.0f, -g.platform_radius, 0.0f);
  g.plat_off[1] = sf::Vector3f( g.platform_radius * kSin120, g.platform_radius * 0.5f, 0.0f);
  g.plat_off[2] = sf::Vector3f(-g.platform_radius * kSin120, g.platform_radius * 0.5f, 0.0f);


  const auto motor = n.declare_parameter<std::vector<double>>("geometry.motor_positions", std::vector<double>());
  if (motor.size() >= 9) {
    for (int i = 0; i < 3; ++i) {
      g.motors[i] = sf::Vector3f(static_cast<float>(motor[i * 3]),
                                 static_cast<float>(motor[i * 3 + 1]),
                                 static_cast<float>(motor[i * 3 + 2]));
    }
  }
  const auto plat_offs = n.declare_parameter<std::vector<double>>("geometry.platform_offsets", std::vector<double>());
  if (plat_offs.size() >= 9) {
    for (int i = 0; i < 3; ++i) {
      g.plat_off[i] = sf::Vector3f(static_cast<float>(plat_offs[i * 3]),
                                   static_cast<float>(plat_offs[i * 3 + 1]),
                                   static_cast<float>(plat_offs[i * 3 + 2]));
    }
  }
  return g;
}

static sf::Vector2f toScreenXY(const sf::Vector3f & p, float cx, float cy) {
  return sf::Vector2f(cx + p.x * kScale, cy - p.y * kScale);
}

static sf::Vector2f toScreenXZ(const sf::Vector3f & p, float cx, float cy) {
  return sf::Vector2f(cx + p.x * kScaleSide, cy - p.z * kScaleSide);
}

static sf::Vector3f limbRadial(int limb) {
  switch (limb) {
    case 0: return sf::Vector3f(0.0f, -1.0f, 0.0f);
    case 1: return sf::Vector3f( kSin120, 0.5f, 0.0f);
    default: return sf::Vector3f(-kSin120, 0.5f, 0.0f);
  }
}

static sf::Vector3f elbowFromAngle(const Geom & g, int limb, float armAngleDeg) {
  const float c = std::cos(armAngleDeg * kPi / 180.0f);
  const float r = g.upper_arm_len * c;
  const sf::Vector3f u = limbRadial(limb);
  return g.motors[limb] + sf::Vector3f(u.x * r, u.y * r, -g.upper_arm_len * std::sin(armAngleDeg * kPi / 180.0f));
}

static sf::Vector3f platformJoint(const Geom & g, int joint, const sf::Vector3f & e) {
  return sf::Vector3f(e.x + g.plat_off[joint].x,
                      e.y + g.plat_off[joint].y,
                      e.z + g.plat_off[joint].z);
}

static sf::Vector3f limbTangent(int limb) {
  switch (limb) {
    case 0: return sf::Vector3f(1.0f, 0.0f, 0.0f);
    case 1: return sf::Vector3f(-0.5f, 0.8660254037844386f, 0.0f);
    default: return sf::Vector3f(-0.5f, -0.8660254037844386f, 0.0f);
  }
}

static sf::Vector3f armUnitDir(int limb, float armAngleDeg) {
  const float c = std::cos(armAngleDeg * kPi / 180.0f);
  const float s = std::sin(armAngleDeg * kPi / 180.0f);
  const sf::Vector3f u = limbRadial(limb);
  return sf::Vector3f(u.x * c, u.y * c, -s);
}

static sf::Vector3f armStandoff(int limb, float armAngleDeg) {
  const float s = std::sin(armAngleDeg * kPi / 180.0f);
  const float c = std::cos(armAngleDeg * kPi / 180.0f);
  const sf::Vector3f u = limbRadial(limb);
  return sf::Vector3f(-u.x * s, -u.y * s, -c);
}

struct LinkPins
{
    sf::Vector3f e, f;
    sf::Vector3f arm;
};

static constexpr float kCrankZeroDeg = 0.0f;
static constexpr float kCrankSign    = -1.0f;

static sf::Vector3f crankPin(int limb, float crank, float servoDeg,
                             const sf::Vector3f & s) {
  const sf::Vector3f u = limbRadial(limb);
  const float rs = s.x * u.x + s.y * u.y;
  const float a = (kCrankZeroDeg + kCrankSign * servoDeg) * kPi / 180.0f;
  return u * (rs + crank * std::cos(a)) +
         sf::Vector3f(0.0f, 0.0f, s.z + crank * std::sin(a));
}

static sf::Vector3f armSocket(const Geom & g, int limb, float armAngleDeg) {
  const sf::Vector3f d = armUnitDir(limb, armAngleDeg);
  return g.motors[limb] + d * g.arm_attach_dist +
         armStandoff(limb, armAngleDeg) * g.arm_attach_offset;
}

static bool armAngleFromCrank(int limb, float servoDeg, const Geom & g,
                              float nearPhi, float & phiOut) {
  const sf::Vector3f u = limbRadial(limb);
  const sf::Vector3f e = crankPin(limb, g.upper_rod_len, servoDeg, g.servos[limb]);
  const float er = e.x * u.x + e.y * u.y;
  const float ez = e.z;

  auto residual = [&](float phiDeg) {
    const sf::Vector3f f = armSocket(g, limb, phiDeg);
    return std::hypot(er - (f.x * u.x + f.y * u.y), ez - f.z) - g.servo_rod_len;
  };

  float best = 0.0f;
  float bestDist = 0.0f;
  bool found = false;
  float prevPhi = -180.0f;
  float prev = residual(prevPhi);
  for (int step = 1; step <= 720; ++step) {
    const float phi = -180.0f + 0.5f * step;
    const float cur = residual(phi);
    if ((prev < 0.0f) != (cur < 0.0f)) {
      float lo = prevPhi, hi = phi;
      for (int k = 0; k < 50; ++k) {
        const float mid = 0.5f * (lo + hi);
        if ((residual(lo) < 0.0f) != (residual(mid) < 0.0f)) hi = mid; else lo = mid;
      }
      const float root = 0.5f * (lo + hi);
      const float d = std::fabs(root - nearPhi);
      if (!found || d < bestDist) { best = root; bestDist = d; found = true; }
    }
    prevPhi = phi;
    prev = cur;
  }
  if (!found) return false;
  phiOut = best;
  return true;
}

static LinkPins servoLinkage(const Geom & g, int limb, float armAngleDeg, float servoDeg) {
  const sf::Vector3f d = armUnitDir(limb, armAngleDeg);
  LinkPins l;
  l.arm = g.motors[limb] + d * g.arm_attach_dist;
  l.f = armSocket(g, limb, armAngleDeg);

  l.e = crankPin(limb, g.upper_rod_len, servoDeg, g.servos[limb]);
  return l;
}

static bool fkFromAngles(const Geom & g, const float angDeg[3], sf::Vector3f & e)
{
  sf::Vector3f c[3];
  for (int i = 0; i < 3; ++i) {
    const sf::Vector3f el = elbowFromAngle(g, i, angDeg[i]);
    c[i] = el - g.plat_off[i];
  }

  const sf::Vector3f A = c[1] - c[0];
  const sf::Vector3f B = c[2] - c[0];
  const float gAA = A.x * A.x + A.y * A.y + A.z * A.z;
  const float gBB = B.x * B.x + B.y * B.y + B.z * B.z;
  const float gAB = A.x * B.x + A.y * B.y + A.z * B.z;
  const float denom = gAA * gBB - gAB * gAB;
  if (denom < 1e-6f) return false;


  const float p = (0.5f * gAA * gBB - 0.5f * gBB * gAB) / denom;
  const float q = (0.5f * gBB * gAA - 0.5f * gAA * gAB) / denom;
  const sf::Vector3f x0(A.x * p + B.x * q, A.y * p + B.y * q, A.z * p + B.z * q);


  const float n2 = denom;
  const float disc = g.lower_arm_len * g.lower_arm_len - (x0.x * x0.x + x0.y * x0.y + x0.z * x0.z);
  if (disc < 0.0f) return false;
  const float lam = std::sqrt(disc / n2);
  const sf::Vector3f n(A.y * B.z - A.z * B.y, A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x);


  const sf::Vector3f lo = c[0] + x0 + n * lam;
  const sf::Vector3f hi = c[0] + x0 - n * lam;
  e = (lo.z < hi.z) ? lo : hi;
  return true;
}

static void drawLine(sf::RenderWindow & w,
                     sf::Vector2f a, sf::Vector2f b,
                     sf::Color color, float thickness = 2.0f) {
  sf::Vector2f d = b - a;
  float len = std::sqrt(d.x * d.x + d.y * d.y);
  if (len < 0.01f) return;
  sf::RectangleShape rect(sf::Vector2f(len, thickness));
  rect.setOrigin(0.0f, thickness * 0.5f);
  rect.setPosition(a);
  rect.setRotation(std::atan2(d.y, d.x) * 180.0f / kPi);
  rect.setFillColor(color);
  w.draw(rect);
}

static void drawDot(sf::RenderWindow & w, sf::Vector2f p,
                    float r, sf::Color fill, sf::Color outline = sf::Color::Transparent) {
  sf::CircleShape c(r);
  c.setOrigin(r, r);
  c.setPosition(p);
  c.setFillColor(fill);
  if (outline != sf::Color::Transparent) {
    c.setOutlineThickness(1.0f);
    c.setOutlineColor(outline);
  }
  w.draw(c);
}

static void drawText(sf::RenderWindow & w, sf::Vector2f pos,
                     const std::string & str, sf::Font & f, unsigned sz, sf::Color c) {
  sf::Text t(str, f, sz);
  t.setPosition(pos);
  t.setFillColor(c);
  w.draw(t);
}

static void drawLabel(sf::RenderWindow & w, sf::Vector2f p, const std::string & str,
                      sf::Font & f, unsigned sz, sf::Color c) {
  drawText(w, sf::Vector2f(p.x + 8.0f, p.y - 15.0f), str, f, sz, c);
}

static sf::Color lighter(sf::Color c, float amount = 0.35f) {
  return sf::Color(static_cast<sf::Uint8>(c.r + (255 - c.r) * amount),
                   static_cast<sf::Uint8>(c.g + (255 - c.g) * amount),
                   static_cast<sf::Uint8>(c.b + (255 - c.b) * amount));
}

static void drawTriangle(sf::RenderWindow & w,
                         const sf::Vector3f & v0, const sf::Vector3f & v1,
                         const sf::Vector3f & v2,
                         float cx, float cy, sf::Color color, float thick = 1.5f) {
  auto p0 = toScreenXY(v0, cx, cy);
  auto p1 = toScreenXY(v1, cx, cy);
  auto p2 = toScreenXY(v2, cx, cy);
  drawLine(w, p0, p1, color, thick);
  drawLine(w, p1, p2, color, thick);
  drawLine(w, p2, p0, color, thick);
}

static void drawForearm(sf::RenderWindow & w, const Geom & g, int limb,
                        const sf::Vector3f & e, const sf::Vector3f & p,
                        sf::Vector2f (*proj)(const sf::Vector3f &, float, float),
                        float cx, float cy, sf::Color color, float thick = 2.0f) {
  const sf::Vector3f t = limbTangent(limb);
  drawLine(w, proj(e + t * g.rod_spread, cx, cy), proj(p + t * g.rod_spread, cx, cy), color, thick);
  drawLine(w, proj(e - t * g.rod_spread, cx, cy), proj(p - t * g.rod_spread, cx, cy), color, thick);
}

static void drawLinkage(sf::RenderWindow & w, const Geom & g, int limb, float armAngleDeg,
                        float servoDeg,
                        sf::Vector2f (*proj)(const sf::Vector3f &, float, float),
                        float cx, float cy, bool outOfClosure = false) {
  const sf::Vector3f s = g.servos[limb];
  const LinkPins l = servoLinkage(g, limb, armAngleDeg, servoDeg);


  const sf::Vector3f postBase(s.x, s.y, 0.0f);
  drawLine(w, proj(postBase, cx, cy), proj(s, cx, cy), sf::Color(90, 90, 105), 5.0f);


  const sf::Color upperCol = outOfClosure ? sf::Color(235, 120, 60) : sf::Color(150, 150, 165);
  const sf::Color lowerCol = outOfClosure ? sf::Color(255, 170, 90) : sf::Color(185, 185, 195);
  drawLine(w, proj(s, cx, cy), proj(l.e, cx, cy), upperCol, 3.0f);


  drawLine(w, proj(l.e, cx, cy), proj(l.f, cx, cy), lowerCol, 2.0f);

  drawLine(w, proj(l.arm, cx, cy), proj(l.f, cx, cy), upperCol, 1.5f);

  drawDot(w, proj(l.e, cx, cy), 2.2f, sf::Color(90, 90, 100));
  drawDot(w, proj(l.f, cx, cy), 2.2f, sf::Color(90, 90, 100));
  if (outOfClosure) {

    drawDot(w, proj(s, cx, cy), 5.0f, sf::Color(255, 90, 60));
  }
}

static bool checkDisplay() {
  const char * display = std::getenv("DISPLAY");
  if (display == nullptr || display[0] == '\0') {
    std::fprintf(stderr,
      "[arm_sim_sfml] ERROR: no graphical display: DISPLAY is not set.\n"
      "[arm_sim_sfml] The 2-D view needs an X server. Run it from a desktop\n"
      "[arm_sim_sfml] session or through scripts/launch_container.sh (which\n"
      "[arm_sim_sfml] forwards DISPLAY/XAUTHORITY), or use a headless launch:\n"
      "[arm_sim_sfml]   ros2 launch arm hardware.launch.py driver:=false\n"
      "[arm_sim_sfml]   ros2 launch arm_gazebo sim_3d.launch.py\n");
    return false;
  }
#if defined(ARM_SIM_HAVE_X11)
  Display * xdisplay = XOpenDisplay(nullptr);
  if (xdisplay == nullptr) {
    std::fprintf(stderr,
      "[arm_sim_sfml] ERROR: cannot connect to X display '%s'.\n"
      "[arm_sim_sfml] Check that the X socket and cookie are available to the\n"
      "[arm_sim_sfml] container (DISPLAY, /tmp/.X11-unix, XAUTHORITY). The host\n"
      "[arm_sim_sfml] helper scripts/launch_container.sh sets these up.\n",
      display);
    return false;
  }
  XCloseDisplay(xdisplay);
#endif
  return true;
}

int main(int argc, char **argv) {
  if (!checkDisplay()) return 1;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeltaArmSim::ArmSimSFMLNode>();

  const Geom geom = loadGeometry(*node);
  const double step = node->declare_parameter<double>("step", 0.01);


  const auto init_pos = node->declare_parameter<std::vector<double>>(
    "sim.initial_pos", std::vector<double>{0.0, 0.0, -0.30});
  node->declare_parameter<bool>("sim.simulate_arrival", true);
  double tx = init_pos.size() == 3 ? init_pos[0] : 0.0;
  double ty = init_pos.size() == 3 ? init_pos[1] : 0.0;
  double tz = init_pos.size() == 3 ? init_pos[2] : -0.30;


  {
    std::string reason;
    if (node->seed_pose(tx, ty, tz, node->ang_, reason)) {
      node->pos_x_ = tx;
      node->pos_y_ = ty;
      node->pos_z_ = tz;
    } else {
      RCLCPP_WARN(node->get_logger(), "initial pose (%.3f, %.3f, %.3f) m is unreachable: %s", tx, ty, tz,
                  reason.c_str());
    }
  }

  sf::RenderWindow window(sf::VideoMode(1200, 600), "Delta Arm 2D Sim");
  window.setFramerateLimit(60);

  std::thread spinner([node]() { rclcpp::spin(node); });

  sf::Font font;
  bool hasFont = font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");


  sf::Clock frame_clock;


  const float halfW = 600.0f;
  const float cxTop  = halfW * 0.5f;
  const float cyTop  = 300.0f;
  const float cxSide = halfW + halfW * 0.5f;
  const float cySide = 176.0f;


  sf::Color limbCol[3] = {
    sf::Color(255, 80, 80),
    sf::Color(80, 255, 80),
    sf::Color(80, 130, 255)};
  std::string limbName[3] = {"leg 0", "leg 1", "leg 2"};


  while (window.isOpen() && rclcpp::ok()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      } else if (event.type == sf::Event::KeyPressed) {
        switch (event.key.code) {
          case sf::Keyboard::W: ty += step; break;
          case sf::Keyboard::S: ty -= step; break;
          case sf::Keyboard::A: tx -= step; break;
          case sf::Keyboard::D: tx += step; break;
          case sf::Keyboard::Z: tz += step; break;
          case sf::Keyboard::X: tz -= step; break;
          case sf::Keyboard::Q: window.close(); break;
          default: break;
        }
        node->send_target(tx, ty, tz);
      }
    }

    window.clear(sf::Color(25, 25, 35));


    drawLine(window, sf::Vector2f(halfW, 0), sf::Vector2f(halfW, 600),
             sf::Color(60, 60, 60), 1.0f);

    if (hasFont) {
      drawText(window, sf::Vector2f(10, 10), "Top View (XY)", font, 16,
               sf::Color(160, 160, 160));
      drawText(window, sf::Vector2f(halfW + 10, 10), "Side View (XZ)", font, 16,
               sf::Color(160, 160, 160));
      drawText(window, sf::Vector2f(10, 30),
               node->feedback_source_active()
                 ? "SOURCE: real servo feedback (arm/MotorFeedback)"
                 : "SOURCE: sim stream (arm/pos)",
               font, 13,
               node->feedback_source_active() ? sf::Color(120, 220, 120) : sf::Color(210, 200, 130));
      drawText(window, sf::Vector2f(10, 46),
               "keys: W/S target y, A/D target x, Z/X target z, Q quit",
               font, 13, sf::Color(160, 160, 160));
      if (node->goal_rejected()) {
        drawText(window, sf::Vector2f(10, 64),
                 "TARGET REJECTED (arm did not move): " + node->last_rejection(),
                 font, 13, sf::Color(255, 120, 120));
      }
    }


    const bool live_fb = node->select_source();
    const float raw_ang[3] = {
      live_fb ? node->fb_ang_[0] : node->ang_[0],
      live_fb ? node->fb_ang_[1] : node->ang_[1],
      live_fb ? node->fb_ang_[2] : node->ang_[2]};
    node->update_display(raw_ang, frame_clock.restart().asSeconds());
    const float * ang = node->display_angles();


    const float ang_tar[3] = {
      node->ang_tar_[0], node->ang_tar_[1], node->ang_tar_[2]};
    const float armTar[3] = {
      node->arm_angle_deg(ang_tar[0], 0),
      node->arm_angle_deg(ang_tar[1], 1),
      node->arm_angle_deg(ang_tar[2], 2)};


    static float last_arm[3] = {60.0f, 60.0f, 60.0f};
    static bool  last_arm_valid[3] = {false, false, false};
    bool out_of_closure[3] = {false, false, false};
    float armAng[3];
    for (int i = 0; i < 3; ++i) {
      float a = 0.0f;
      if (armAngleFromCrank(i, ang[i], geom, last_arm[i], a)) {
        last_arm[i] = a;
        last_arm_valid[i] = true;
      } else {
        out_of_closure[i] = true;


        a = last_arm_valid[i] ? last_arm[i] : 60.0f;
      }
      armAng[i] = a;
    }

    sf::Vector3f e(static_cast<float>(node->pos_x_) * 1000.0f,
                   static_cast<float>(node->pos_y_) * 1000.0f,
                   static_cast<float>(node->pos_z_) * 1000.0f);


    static sf::Vector3f last_fk(0.0f, 0.0f, 0.0f);
    static bool last_fk_valid = false;
    if (live_fb) {
      sf::Vector3f e_try;
      const bool closure_ok = !(out_of_closure[0] || out_of_closure[1] || out_of_closure[2]);
      if (closure_ok && fkFromAngles(geom, armAng, e_try)) {
        e = e_try;
        last_fk = e_try;
        last_fk_valid = true;
      } else if (last_fk_valid) {
        e = last_fk;
      }
    }


    const sf::Vector3f tgt(static_cast<float>(node->tar_x_) * 1000.0f,
                           static_cast<float>(node->tar_y_) * 1000.0f,
                           static_cast<float>(node->tar_z_) * 1000.0f);
    const sf::Color tgtCol(255, 90, 220);

    sf::Vector3f elbow[3], plat[3];
    for (int i = 0; i < 3; ++i) {
      elbow[i] = elbowFromAngle(geom, i, armAng[i]);
      plat[i] = platformJoint(geom, i, e);
    }


    drawTriangle(window, geom.servos[0], geom.servos[1], geom.servos[2],
                 cxTop, cyTop, sf::Color(110, 110, 125), 1.5f);


    for (int i = 0; i < 3; ++i) {
      drawLinkage(window, geom, i, armAng[i], ang[i], toScreenXY, cxTop, cyTop, out_of_closure[i]);
    }


    drawTriangle(window, plat[0], plat[1], plat[2], cxTop, cyTop,
                 sf::Color(120, 160, 220), 1.5f);


    for (int i = 0; i < 3; ++i) {
      drawLine(window, toScreenXY(geom.motors[i], cxTop, cyTop),
               toScreenXY(elbow[i], cxTop, cyTop), limbCol[i], 3.5f);
      drawForearm(window, geom, i, elbow[i], plat[i], toScreenXY, cxTop, cyTop,
                  lighter(limbCol[i]), 2.0f);
    }


    for (int i = 0; i < 3; ++i) {
      drawDot(window, toScreenXY(geom.servos[i], cxTop, cyTop), 4.5f, limbCol[i], sf::Color(20, 20, 20));
      drawDot(window, toScreenXY(geom.motors[i], cxTop, cyTop), 4.0f, sf::Color(215, 215, 215), limbCol[i]);
      drawDot(window, toScreenXY(elbow[i], cxTop, cyTop), 3.5f, limbCol[i]);
      drawDot(window, toScreenXY(plat[i], cxTop, cyTop), 3.5f, limbCol[i]);
      if (hasFont) {
        drawLabel(window, toScreenXY(geom.servos[i], cxTop, cyTop), "S" + std::to_string(i), font, 11, limbCol[i]);
        drawLabel(window, toScreenXY(geom.motors[i], cxTop, cyTop), "J" + std::to_string(i), font, 11, limbCol[i]);
        drawLabel(window, toScreenXY(elbow[i], cxTop, cyTop), "E" + std::to_string(i), font, 11, limbCol[i]);
        drawLabel(window, toScreenXY(plat[i], cxTop, cyTop), "P" + std::to_string(i), font, 11, limbCol[i]);


        char thb[32];
        std::snprintf(thb, sizeof(thb), "\u03b8%d=%.0f\u00b0", i, ang[i]);
        const sf::Color thc = (ang[i] < -0.5f || ang[i] > 145.5f) ? sf::Color(255, 200, 80) : limbCol[i];
        drawText(window, toScreenXY(geom.servos[i], cxTop, cyTop) + sf::Vector2f(9.0f, 2.0f),
                 thb, font, 10, thc);
      }
    }
    drawDot(window, toScreenXY(e, cxTop, cyTop), 5.0f, sf::Color::Cyan);
    if (hasFont) drawLabel(window, toScreenXY(e, cxTop, cyTop), "EE", font, 11, sf::Color::Cyan);


    drawLine(window, toScreenXY(e, cxTop, cyTop), toScreenXY(tgt, cxTop, cyTop),
             sf::Color(255, 90, 220, 110), 1.0f);
    drawDot(window, toScreenXY(tgt, cxTop, cyTop), 5.0f, sf::Color::Transparent, tgtCol);
    if (hasFont) drawLabel(window, toScreenXY(tgt, cxTop, cyTop), "TGT", font, 11, tgtCol);


    drawLine(window,
             sf::Vector2f(cxSide - geom.base_radius * kScaleSide, cySide),
             sf::Vector2f(cxSide + geom.base_radius * kScaleSide, cySide),
             sf::Color(60, 60, 60), 1.0f);


    for (int i = 0; i < 3; ++i) {
      drawLinkage(window, geom, i, armAng[i], ang[i], toScreenXZ, cxSide, cySide, out_of_closure[i]);
    }


    for (int i = 0; i < 3; ++i) {
      drawLine(window, toScreenXZ(geom.motors[i], cxSide, cySide),
               toScreenXZ(elbow[i], cxSide, cySide), limbCol[i], 3.5f);
      drawForearm(window, geom, i, elbow[i], plat[i], toScreenXZ, cxSide, cySide,
                  lighter(limbCol[i]), 2.0f);
    }


    for (int i = 0; i < 3; ++i) {
      drawDot(window, toScreenXZ(geom.servos[i], cxSide, cySide), 4.5f, limbCol[i], sf::Color(20, 20, 20));
      drawDot(window, toScreenXZ(geom.motors[i], cxSide, cySide), 4.0f, sf::Color(215, 215, 215), limbCol[i]);
      drawDot(window, toScreenXZ(elbow[i], cxSide, cySide), 3.5f, limbCol[i]);
      drawDot(window, toScreenXZ(plat[i], cxSide, cySide), 3.5f, limbCol[i]);
      if (hasFont) {
        drawLabel(window, toScreenXZ(geom.servos[i], cxSide, cySide), "S" + std::to_string(i), font, 11, limbCol[i]);
        drawLabel(window, toScreenXZ(elbow[i], cxSide, cySide), "E" + std::to_string(i), font, 11, limbCol[i]);
      }
    }
    drawDot(window, toScreenXZ(e, cxSide, cySide), 5.0f, sf::Color::Cyan);


    drawLine(window, toScreenXZ(e, cxSide, cySide), toScreenXZ(tgt, cxSide, cySide),
             sf::Color(255, 90, 220, 110), 1.0f);
    drawDot(window, toScreenXZ(tgt, cxSide, cySide), 5.0f, sf::Color::Transparent, tgtCol);


    if (hasFont) {
      drawText(window, sf::Vector2f(halfW + 10, 470), "Colour = limb: red=leg 0, green=leg 1, blue=leg 2", font, 12,
               sf::Color(150, 150, 150));
      drawText(window, sf::Vector2f(halfW + 10, 486), "S = servo   J = arm actuation joint   E = elbow   P = platform joint   EE = end effector", font, 12,
               sf::Color(150, 150, 150));
      drawText(window, sf::Vector2f(halfW + 10, 502), "grey = servo body + upper rod + lower rod; lower arm (elbow>platform) = 2 parallel bars", font, 12,
               sf::Color(150, 150, 150));
      drawText(window, sf::Vector2f(halfW + 10, 518), "thick = upper arm, thin = lower arm (parallel bars)", font, 12,
               sf::Color(150, 150, 150));
      drawText(window, sf::Vector2f(halfW + 10, 534), "servo angle \u03b8 ~ [0,145]\u00b0; horn drawn AT \u03b8 from horizon, arm \u03c6 solved from it", font, 12,
               sf::Color(150, 150, 150));


      char thBuf[160];
      std::snprintf(thBuf, sizeof(thBuf),
                    "motor \u03b8 cur=(%5.1f, %5.1f, %5.1f)\u00b0  tar=(%5.1f, %5.1f, %5.1f)\u00b0",
                    ang[0], ang[1], ang[2], ang_tar[0], ang_tar[1], ang_tar[2]);
      drawText(window, sf::Vector2f(halfW + 10, 554), thBuf, font, 12,
               sf::Color(210, 210, 210));
      char phBuf[160];
      std::snprintf(phBuf, sizeof(phBuf),
                    "arm   \u03c6 cur=(%5.1f, %5.1f, %5.1f)\u00b0  tar=(%5.1f, %5.1f, %5.1f)\u00b0",
                    armAng[0], armAng[1], armAng[2], armTar[0], armTar[1], armTar[2]);
      drawText(window, sf::Vector2f(halfW + 10, 572), phBuf, font, 12,
               sf::Color(140, 200, 230));
      char buf[200];
      std::snprintf(buf, sizeof(buf),
                    "pos=(%.1f, %.1f, %.1f)  tgt=(%.1f, %.1f, %.1f) mm",
                    e.x, e.y, e.z, tgt.x, tgt.y, tgt.z);
      drawText(window, sf::Vector2f(halfW + 10, 590), buf, font, 12,
               sf::Color(180, 180, 180));
      if (out_of_closure[0] || out_of_closure[1] || out_of_closure[2]) {
        std::string limbs;
        for (int i = 0; i < 3; ++i) {
          if (!out_of_closure[i]) continue;
          char limbBuf[64];
          std::snprintf(limbBuf, sizeof(limbBuf), "%sleg %d \u03b8=%5.1f\u00b0",
                        limbs.empty() ? "" : ", ", i, ang[i]);
          limbs += limbBuf;
        }
        drawText(window, sf::Vector2f(halfW + 10, 608),
                 "ROD CANNOT REACH (orange): " + limbs,
                 font, 12, sf::Color(255, 150, 90));
        drawText(window, sf::Vector2f(halfW + 10, 624),
                 "arm \u03c6 above is the last reachable solve; the horn still shows the true \u03b8",
                 font, 12, sf::Color(200, 130, 90));
      }
    }

    window.display();
  }

  rclcpp::shutdown();
  if (spinner.joinable()) spinner.join();
  return 0;
}