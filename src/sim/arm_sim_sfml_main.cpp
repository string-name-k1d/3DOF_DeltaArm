// arm_sim_sfml_main.cpp – 2-D SFML delta-arm visualiser.
//
// Subscribes to arm/pos (ArmPosition) and draws:
//   left half  – top view (XY plane)
//   right half – side view (XZ plane, looking along Y)
//
// Mechanism geometry is loaded from ROS parameters ("geometry.*", shared with
// the controller via arm_params.yaml) instead of being hardcoded. The base
// triangle is drawn THROUGH the three SERVO output shafts (the plate's
// vertices). The servos are mounted UNDER the base plate: each drives its arm
// through a two-rod linkage (single upper rod on the shaft + single lower rod)
// running below the plate up to a point at fixed distance from the inner
// actuation joint (radius t) on the upper arm; the linkage rotates by the arm
// angle, and a readout shows each servo angle (nominal range [0,145] deg,
// amber when outside). The LOWER ARM (elbow -> platform joint) is drawn as two
// parallel bars. Joints and rods are colour-coded per limb: red = leg 0,
// green = leg 1, blue = leg 2.

#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <SFML/Graphics.hpp>

#include "arm/arm_sim_sfml.hpp"

// ── Numeric constants used for the derived joint geometry ──────────────────
static constexpr float kSqrt3  = 1.7320508075688772f;
static constexpr float kTan30  = 0.5773502691896258f;   // 1/√3
static constexpr float kSin120 = 0.8660254037844386f;   // √3/2
static constexpr float kCos120 = -0.5f;
static constexpr float kPi     = 3.141592653589793f;

// Pixel scale: 1 mm = this many pixels.
static constexpr float kScale = 1.25f;   // mm -> px; fits servo plate (150) + elbow reach (205)

// ── Loaded mechanism geometry ────────────────────────────────────────────────
struct Geom
{
  float base_radius = 150.0f;     // circumradius, base-plate triangle (mm)
  float platform_radius = 60.0f;  // circumradius, platform-joint triangle (mm)
  float upper_arm_len = 160.0f;   // arm actuation joint -> elbow (mm)
  float lower_arm_len = 320.0f;   // elbow -> platform joint rod (mm)

  float base_side = 0.0f;         // base triangle side length (f)
  float t = 0.0f;                 // arm actuation-joint offset from centre axis (mm)

  // Servo-linkage geometry (drives the "fake" arm joints). Mechanism per arm:
  //   servo shaft -> "upper" rod (a SINGLE bar fixed on the shaft)
  //               -> "lower" rod (a SINGLE bar)
  //               -> the upper arm, at a point at fixed distance from the base
  //                  (actuation) joint. The LOWER ARM (elbow -> platform joint)
  //                  is drawn as 2 PARALLEL bars. Names match arm_params.yaml.
  float servo_radius = 150.0f;      // radius of the servo output shafts (mm)
  float servo_z = -25.0f;           // servo height BELOW the pivot plane: the
                                    // servo mounts under the base plate and the
                                    // whole linkage runs below it, up to the
                                    // arm's attach point (matches the CAD photo).
  float upper_rod_len = 60.0f;      // "upper rod": single bar fixed to the servo
                                    // shaft, length (mm); points inward to the hub
  float arm_attach_dist = 30.0f;    // fixed distance from the arm's base joint to
                                    // the lower-rod attach point, along the arm (mm)
  float rod_spread = 8.0f;          // lateral separation of the two parallel bars
                                    // of each LOWER ARM (visual only, ±mm)

  sf::Vector3f motors[3];         // arm actuation joints, world frame (mm)
  sf::Vector3f plat_off[3];       // platform-joint offsets from effector centre (mm)
  sf::Vector3f servos[3];         // servo output-shaft positions, world frame (mm)
};

// Load geometry from ROS parameters (declare with defaults so it works even
// without a params file).
static Geom loadGeometry(rclcpp::Node & n)
{
  Geom g;
  g.base_radius = static_cast<float>(n.declare_parameter<double>("geometry.base_radius", 150.0));
  g.platform_radius = static_cast<float>(n.declare_parameter<double>("geometry.platform_radius", 60.0));
  g.upper_arm_len = static_cast<float>(n.declare_parameter<double>("geometry.upper_arm_len", 160.0));
  g.lower_arm_len = static_cast<float>(n.declare_parameter<double>("geometry.lower_arm_len", 320.0));

  g.base_side = g.base_radius * kSqrt3;                     // f
  const float platformSide = g.platform_radius * kSqrt3;    // e
  g.t = (g.base_side - platformSide) * kTan30 / 2.0f;

  // Derived joint positions: an equilateral set spaced 120 deg apart, matching
  // the delta-arm FK / Gazebo model. Leg 0 points along -Y. The effector
  // triangle is INVERTED (apex-down): each platform joint sits on the SAME
  // radial as its motor, which is the orientation the IK/FK solve for.
  g.motors[0] = sf::Vector3f(0.0f, -g.t, 0.0f);
  g.motors[1] = sf::Vector3f( g.t * kSin120,  g.t * 0.5f, 0.0f);
  g.motors[2] = sf::Vector3f(-g.t * kSin120,  g.t * 0.5f, 0.0f);

  // Actual servo output shafts: mounted under the base-plate triangle vertices
  // (same radial as each arm actuation joint). The servo linkage is a
  // crank + 2 parallel bars running below the plate up to the arm's attach
  // point (a fixed distance from the arm's base joint).
  g.servo_radius     = static_cast<float>(n.declare_parameter<double>("geometry.servo_radius", g.base_radius));
  g.servo_z          = static_cast<float>(n.declare_parameter<double>("geometry.servo_z", -25.0));
  g.upper_rod_len    = static_cast<float>(n.declare_parameter<double>("geometry.upper_rod_len", 60.0));
  g.arm_attach_dist  = static_cast<float>(n.declare_parameter<double>("geometry.arm_attach_dist", 30.0));
  g.rod_spread       = static_cast<float>(n.declare_parameter<double>("geometry.rod_spread", 8.0));
  g.servos[0] = sf::Vector3f(0.0f, -g.servo_radius, g.servo_z);
  g.servos[1] = sf::Vector3f( g.servo_radius * kSin120,  g.servo_radius * 0.5f, g.servo_z);
  g.servos[2] = sf::Vector3f(-g.servo_radius * kSin120,  g.servo_radius * 0.5f, g.servo_z);

  g.plat_off[0] = sf::Vector3f(0.0f, -g.platform_radius, 0.0f);
  g.plat_off[1] = sf::Vector3f( g.platform_radius * kSin120, g.platform_radius * 0.5f, 0.0f);
  g.plat_off[2] = sf::Vector3f(-g.platform_radius * kSin120, g.platform_radius * 0.5f, 0.0f);

  // Optional explicit overrides (mm, world frame, flattened x/y/z per leg).
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

// ── Coordinate helpers ────────────────────────────────────────────────────────

// mm → screen pixel (top-view: x → horizontal, y → vertical-up).
static sf::Vector2f toScreenXY(const sf::Vector3f & p, float cx, float cy) {
  return sf::Vector2f(cx + p.x * kScale, cy - p.y * kScale);
}

// mm → screen pixel (side-view: x → horizontal, z → vertical-up).
static sf::Vector2f toScreenXZ(const sf::Vector3f & p, float cx, float cy) {
  return sf::Vector2f(cx + p.x * kScale, cy - p.z * kScale);
}

// ── Limb geometry ─────────────────────────────────────────────────────────────
// Outward radial direction of each limb (world frame): from the centre axis to
// its servo pivot.
static sf::Vector3f limbRadial(int limb) {
  switch (limb) {
    case 0: return sf::Vector3f(0.0f, -1.0f, 0.0f);
    case 1: return sf::Vector3f( kSin120, 0.5f, 0.0f);
    default: return sf::Vector3f(-kSin120, 0.5f, 0.0f);
  }
}

// Elbow (upper-arm end) from a motor angle (degrees).
// The upper arm sweeps downward out of the horizontal: outward component
// rf·cos(a) along the limb's radial, plus -rf·sin(a) in z.
static sf::Vector3f elbowFromAngle(const Geom & g, int limb, float motorAngleDeg) {
  const float c = std::cos(motorAngleDeg * kPi / 180.0f);
  const float r = g.upper_arm_len * c;
  const sf::Vector3f u = limbRadial(limb);
  return g.motors[limb] + sf::Vector3f(u.x * r, u.y * r, -g.upper_arm_len * std::sin(motorAngleDeg * kPi / 180.0f));
}

// Platform joint from the end-effector centre (+ platform offset).
static sf::Vector3f platformJoint(const Geom & g, int joint, const sf::Vector3f & e) {
  return sf::Vector3f(e.x + g.plat_off[joint].x,
                      e.y + g.plat_off[joint].y,
                      e.z + g.plat_off[joint].z);
}

// Horizontal tangent to the limb circle (perpendicular to the radial in the
// base XY plane). Used to separate the two parallel bars of each LOWER ARM
// laterally so they are visibly distinct in BOTH the top and side views.
static sf::Vector3f limbTangent(int limb) {
  switch (limb) {
    case 0: return sf::Vector3f(1.0f, 0.0f, 0.0f);
    case 1: return sf::Vector3f(-0.5f, 0.8660254037844386f, 0.0f);
    default: return sf::Vector3f(-0.5f, -0.8660254037844386f, 0.0f);
  }
}

// Unit direction of the upper arm from its actuation joint: radial·cos(a)
// plus -sin(a)·z (a=0 -> straight out along the radial, growing = downward).
static sf::Vector3f armUnitDir(int limb, float motorAngleDeg) {
  const float c = std::cos(motorAngleDeg * kPi / 180.0f);
  const float s = std::sin(motorAngleDeg * kPi / 180.0f);
  const sf::Vector3f u = limbRadial(limb);
  return sf::Vector3f(u.x * c, u.y * c, -s);
}

// Servo-linkage points: "upper" rod is a SINGLE bar fixed on the servo shaft
// s, ending at e; "lower" rod is a SINGLE bar from e to the arm's attach point
// f, which sits on the upper arm at a fixed distance (arm_attach_dist) from
// its base joint a. (The lower ARM, elbow -> platform, is a separate pair of
// parallel bars drawn in the limb loops.)
struct LinkPins
{
  sf::Vector3f e, f;
};

static LinkPins servoLinkage(const Geom & g, int limb, float motorAngleDeg) {
  const sf::Vector3f s = g.servos[limb];
  const sf::Vector3f a = g.motors[limb];
  const sf::Vector3f d = armUnitDir(limb, motorAngleDeg);
  // Upper rod far end: INWARD from the shaft (opposite the arm direction), so
  // the linkage stays between the servo and the hub (never reaches the elbow).
  LinkPins l;
  l.e = s - d * g.upper_rod_len;
  // Lower-rod attach point on the upper arm: fixed distance from the base joint.
  l.f = a + d * g.arm_attach_dist;
  return l;
}

// ── Drawing helpers ───────────────────────────────────────────────────────────

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

// Small joint label offset slightly up-right of the point.
static void drawLabel(sf::RenderWindow & w, sf::Vector2f p, const std::string & str,
                      sf::Font & f, unsigned sz, sf::Color c) {
  drawText(w, sf::Vector2f(p.x + 8.0f, p.y - 15.0f), str, f, sz, c);
}

// A lighter tint of a colour (for the lower-arm bars).
static sf::Color lighter(sf::Color c, float amount = 0.35f) {
  return sf::Color(static_cast<sf::Uint8>(c.r + (255 - c.r) * amount),
                   static_cast<sf::Uint8>(c.g + (255 - c.g) * amount),
                   static_cast<sf::Uint8>(c.b + (255 - c.b) * amount));
}

// Triangle outline (top view) through three world points.
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

// The LOWER ARM (elbow -> platform joint) drawn as 2 PARALLEL bars, offset
// laterally along the horizontal tangent by rod_spread.
static void drawForearm(sf::RenderWindow & w, const Geom & g, int limb,
                        const sf::Vector3f & e, const sf::Vector3f & p,
                        sf::Vector2f (*proj)(const sf::Vector3f &, float, float),
                        float cx, float cy, sf::Color color, float thick = 2.0f) {
  const sf::Vector3f t = limbTangent(limb);
  drawLine(w, proj(e + t * g.rod_spread, cx, cy), proj(p + t * g.rod_spread, cx, cy), color, thick);
  drawLine(w, proj(e - t * g.rod_spread, cx, cy), proj(p - t * g.rod_spread, cx, cy), color, thick);
}

// Draw one limb's servo linkage in any view: servo body under the plate, the
// single UPPER rod on the shaft, and the single LOWER rod up to the arm
// attach point. `proj` maps world-mm to screen pixels for that view.
static void drawLinkage(sf::RenderWindow & w, const Geom & g, int limb, float motorAngleDeg,
                        sf::Vector2f (*proj)(const sf::Vector3f &, float, float),
                        float cx, float cy) {
  const sf::Vector3f s = g.servos[limb];
  const LinkPins l = servoLinkage(g, limb, motorAngleDeg);

  // Servo body: the servo is mounted UNDER the base plate, so its housing
  // hangs from the plate underside (z=0) down to the shaft plane.
  const sf::Vector3f postBase(s.x, s.y, 0.0f);
  drawLine(w, proj(postBase, cx, cy), proj(s, cx, cy), sf::Color(90, 90, 105), 5.0f);
  // UPPER rod: the single bar fixed to the servo shaft, pointing inward to the
  // hub (so it stays between the servo and the arm, never out to the elbow).
  drawLine(w, proj(s, cx, cy), proj(l.e, cx, cy), sf::Color(150, 150, 165), 3.0f);
  // LOWER rod: the single bar from the upper rod's far end to the arm's attach
  // point (at fixed distance from the arm's base joint).
  drawLine(w, proj(l.e, cx, cy), proj(l.f, cx, cy), sf::Color(185, 185, 195), 2.0f);
  // Pins.
  drawDot(w, proj(l.e, cx, cy), 2.2f, sf::Color(90, 90, 100));
  drawDot(w, proj(l.f, cx, cy), 2.2f, sf::Color(90, 90, 100));
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeltaArmSim::ArmSimSFMLNode>();
  node->enable_position_streaming();

  const Geom geom = loadGeometry(*node);
  const double step = node->declare_parameter<double>("step", 0.01);

  // Initial target = the shared sim.initial_pos (also used by the controller),
  // so the keys move relative to a valid starts pose.
  const auto init_pos = node->declare_parameter<std::vector<double>>(
    "sim.initial_pos", std::vector<double>{0.0, 0.0, -0.30});
  node->declare_parameter<bool>("sim.simulate_arrival", true);
  double tx = init_pos.size() == 3 ? init_pos[0] : 0.0;
  double ty = init_pos.size() == 3 ? init_pos[1] : 0.0;
  double tz = init_pos.size() == 3 ? init_pos[2] : -0.30;

  sf::RenderWindow window(sf::VideoMode(1200, 600), "Delta Arm 2D Sim");
  window.setFramerateLimit(60);

  std::thread spinner([node]() { rclcpp::spin(node); });

  sf::Font font;
  bool hasFont = font.loadFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");

  // Layout: left half = top view, right half = side view.
  const float halfW = 600.0f;
  const float cxTop  = halfW * 0.5f;
  const float cyTop  = 300.0f;
  const float cxSide = halfW + halfW * 0.5f;
  const float cySide = 300.0f;

  // Limb colours: red = 0, green = 1, blue = 2.
  sf::Color limbCol[3] = {
    sf::Color(255, 80, 80),    // red
    sf::Color(80, 255, 80),    // green
    sf::Color(80, 130, 255)};  // blue
  std::string limbName[3] = {"leg 0", "leg 1", "leg 2"};

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      } else if (event.type == sf::Event::KeyPressed) {
        switch (event.key.code) {
          case sf::Keyboard::W: tz += step; break;
          case sf::Keyboard::S: tz -= step; break;
          case sf::Keyboard::A: ty -= step; break;
          case sf::Keyboard::D: ty += step; break;
          case sf::Keyboard::Q: window.close(); break;
          default: break;
        }
        node->send_target(tx, ty, tz);
      }
    }

    window.clear(sf::Color(25, 25, 35));

    // ── Divider + titles ────────────────────────────────────────────────
    drawLine(window, sf::Vector2f(halfW, 0), sf::Vector2f(halfW, 600),
             sf::Color(60, 60, 60), 1.0f);

    if (hasFont) {
      drawText(window, sf::Vector2f(10, 10), "Top View (XY)", font, 16,
               sf::Color(160, 160, 160));
      drawText(window, sf::Vector2f(halfW + 10, 10), "Side View (XZ)", font, 16,
               sf::Color(160, 160, 160));
    }

    // ── Read arm state ──────────────────────────────────────────────────
    const sf::Vector3f e(static_cast<float>(node->pos_x_) * 1000.0f,  // m → mm
                         static_cast<float>(node->pos_y_) * 1000.0f,
                         static_cast<float>(node->pos_z_) * 1000.0f);
    const float ang[3] = {node->ang_[0], node->ang_[1], node->ang_[2]};

    sf::Vector3f elbow[3], plat[3];
    for (int i = 0; i < 3; ++i) {
      elbow[i] = elbowFromAngle(geom, i, ang[i]);
      plat[i] = platformJoint(geom, i, e);
    }

    // ══════════════════════════════════════════════════════════════════════
    //  TOP VIEW  (XY plane – looking down Z)
    // ══════════════════════════════════════════════════════════════════════

    // Base plate triangle drawn THROUGH the actual servo output shafts (the
    // plate's vertices). Each arm is driven from its own servo via a 2-rod
    // linkage (upper rod + lower rod) down to the arm's attach point.
    drawTriangle(window, geom.servos[0], geom.servos[1], geom.servos[2],
                 cxTop, cyTop, sf::Color(110, 110, 125), 1.5f);

    // Servo + two-rod linkage for each limb (behind the arms).
    for (int i = 0; i < 3; ++i) {
      drawLinkage(window, geom, i, ang[i], toScreenXY, cxTop, cyTop);
    }

    // Platform triangle (from end-effector centre).
    drawTriangle(window, plat[0], plat[1], plat[2], cxTop, cyTop,
                 sf::Color(120, 160, 220), 1.5f);

    // Upper arms (thick, limb colour) and lower arms (2 parallel bars, lighter).
    for (int i = 0; i < 3; ++i) {
      drawLine(window, toScreenXY(geom.motors[i], cxTop, cyTop),
               toScreenXY(elbow[i], cxTop, cyTop), limbCol[i], 3.5f);
      drawForearm(window, geom, i, elbow[i], plat[i], toScreenXY, cxTop, cyTop,
                  lighter(limbCol[i]), 2.0f);
    }

    // Joint dots + labels (top view).
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

        // Servo-angle readout next to each servo (nominal range [0,145] deg;
        // amber when the commanded angle leaves that range).
        char thb[32];
        std::snprintf(thb, sizeof(thb), "\u03b8%d=%.0f\u00b0", i, ang[i]);
        const sf::Color thc = (ang[i] < -0.5f || ang[i] > 145.5f) ? sf::Color(255, 200, 80) : limbCol[i];
        drawText(window, toScreenXY(geom.servos[i], cxTop, cyTop) + sf::Vector2f(9.0f, 2.0f),
                 thb, font, 10, thc);
      }
    }
    drawDot(window, toScreenXY(e, cxTop, cyTop), 5.0f, sf::Color::Cyan);
    if (hasFont) drawLabel(window, toScreenXY(e, cxTop, cyTop), "EE", font, 11, sf::Color::Cyan);

    // ══════════════════════════════════════════════════════════════════════
    //  SIDE VIEW  (XZ plane – looking along Y, x → horizontal, z → up)
    // ══════════════════════════════════════════════════════════════════════

    // Ground line at z = 0.
    drawLine(window,
             sf::Vector2f(cxSide - geom.base_radius * kScale, cySide),
             sf::Vector2f(cxSide + geom.base_radius * kScale, cySide),
             sf::Color(60, 60, 60), 1.0f);

    // Servo linkage (side view): servo body below the ground line, single upper
    // rod on the shaft, single lower rod up to the arm attach point.
    for (int i = 0; i < 3; ++i) {
      drawLinkage(window, geom, i, ang[i], toScreenXZ, cxSide, cySide);
    }

    // Upper arms : actuation joint → elbow (projected XZ).
    for (int i = 0; i < 3; ++i) {
      drawLine(window, toScreenXZ(geom.motors[i], cxSide, cySide),
               toScreenXZ(elbow[i], cxSide, cySide), limbCol[i], 3.5f);
      drawForearm(window, geom, i, elbow[i], plat[i], toScreenXZ, cxSide, cySide,
                  lighter(limbCol[i]), 2.0f);
    }

    // Joint dots + labels (side view).
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

    // ── Legend (right half, below the side view) ─────────────────────────
    if (hasFont) {
      drawText(window, sf::Vector2f(halfW + 10, 486), "Colour = limb: red=leg 0, green=leg 1, blue=leg 2", font, 12,
               sf::Color(150, 150, 150));
      drawText(window, sf::Vector2f(halfW + 10, 504), "S = servo   J = arm actuation joint   E = elbow   P = platform joint   EE = end effector", font, 12,
               sf::Color(150, 150, 150));
      drawText(window, sf::Vector2f(halfW + 10, 522), "grey = servo body + upper rod + lower rod; lower arm (elbow>platform) = 2 parallel bars", font, 12,
               sf::Color(150, 150, 150));
      drawText(window, sf::Vector2f(halfW + 10, 540), "thick = upper arm, thin = lower arm (parallel bars)", font, 12,
               sf::Color(150, 150, 150));
      drawText(window, sf::Vector2f(halfW + 10, 558), "servo angle \u03b8 ~ [0,145]\u00b0 (amber = out of range)", font, 12,
               sf::Color(150, 150, 150));
      char buf[160];
      std::snprintf(buf, sizeof(buf), "pos=(%.1f, %.1f, %.1f) mm  angles=(%.1f, %.1f, %.1f) deg",
                    e.x, e.y, e.z, ang[0], ang[1], ang[2]);
      drawText(window, sf::Vector2f(halfW + 10, 580), buf, font, 12,
               sf::Color(180, 180, 180));
    }

    window.display();
  }

  rclcpp::shutdown();
  if (spinner.joinable()) spinner.join();
  return 0;
}