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
// through a two-rod linkage (single crank on the shaft + one RIGID rod of
// constant length) running below the plate up to a point near the ELBOW of the
// upper arm (well beyond the motor shaft's radius). The crank points OUTWARD
// (along the arm direction), and its pin is solved each frame so the connecting
// rod keeps a FIXED length (true 4-bar); its motor-side joint therefore sits
// further away from the platform than an inward crank would.
// The linkage rotates by the arm angle, and a readout shows each servo angle (nominal range [0,145] deg,
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

// ── Numeric constants used for the derived joint geometry ────────────────────
static constexpr float kSin120 = 0.8660254037844386f;   // √3/2
static constexpr float kCos120 = -0.5f;
static constexpr float kPi     = 3.141592653589793f;
static constexpr float kTwoPi  = 6.283185307179586f;
static constexpr float kTwoPiOver3 = 2.0943951023931953f; // 120 deg
static constexpr float kRadToDeg   = 57.29577951308232f;  // 180/π

// Pixel scale: 1 mm = this many pixels.
static constexpr float kScale = 1.25f;   // mm -> px; fits servo plate (150) + elbow reach (205)
// The side view needs its own scale: the arm reaches ~z = -340 mm, which at
// kScale lands 725 px below the z=0 line and off the bottom of the 600 px
// window. 0.80 px/mm keeps the full z envelope (+120 .. -340 mm) inside the
// y = 80..450 band, which is above the legend that starts at y = 470.
static constexpr float kScaleSide = 0.80f;

// ── Loaded mechanism geometry ────────────────────────────────────────────────
struct Geom
{
  float base_radius = 100.0f;     // circumradius, base-plate triangle (mm)
  float platform_radius = 32.5f;  // circumradius, platform-joint triangle (mm)
  float upper_arm_len = 120.0f;   // arm actuation joint -> elbow (mm)
  float lower_arm_len = 240.0f;   // elbow -> platform joint rod (mm)

  float t = 0.0f;                 // arm actuation-joint (shoulder pivot) radius from
                                  // the centre axis (mm): the shoulders sit ON the
                                  // base-plate corner circle (== base_radius).

  // Servo-linkage geometry (drives the arm through a 4-bar). Mechanism per arm:
  //   servo shaft -> servo horn (upper_rod_len, fixed on the shaft)
  //                -> servo rod   (servo_rod_len, RIGID, constant length)
  //                -> a bracket on the upper arm (arm_attach_dist along the arm
  //                   axis + arm_attach_offset perpendicular standoff).
  // These lengths are used only to DRAW the linkage (crank pin / socket); the
  // motor -> arm angle conversion comes from the shared DeltaArm::Arm, so the
  // arm angle drawn here is the same one the controller's forward estimate uses.
  float servo_radius = 57.65f;      // radius of the servo output shafts (mm)
  float servo_z = -22.5f;           // servo height BELOW the pivot plane (mm):
                                    // the servo mounts under the base plate.
  float upper_rod_len = 60.0f;      // horn: single bar fixed to the servo shaft
                                    // (short motor crank), length (mm) = link a
  float servo_rod_len = 35.0f;      // RIGID connecting rod (horn -> arm attach
                                    // point f); link b (mm)
  float arm_attach_dist = 68.5f;    // from the arm's base joint to the rod's
                                    // attach point along the arm (mm): link c =
                                    // hypot(arm_attach_dist, arm_attach_offset)
  float arm_attach_offset = 20.5f;  // PERPENDICULAR standoff of the lower rod's
                                    // arm-side joint from the arm axis (mm)
  float rod_spread = 8.0f;          // lateral separation of the two parallel bars
                                    // of each LOWER ARM (visual only, ±mm)

  sf::Vector3f motors[3];         // arm actuation joints, world frame (mm)
  sf::Vector3f plat_off[3];       // platform-joint offsets from effector centre (mm)
  sf::Vector3f servos[3];         // servo output-shaft positions, world frame (mm)
};

// Load geometry from ROS parameters (declare with defaults so it works even
// without a params file).
// The kinematics-related geometry.* parameters are declared by
// ArmSimSFMLNode::configureKinematics() (it needs them to build the shared
// DeltaArm that the renderer now uses). Declaring them a second time here would
// throw ParameterAlreadyDeclaredException at startup, so read them if they are
// already there and only declare the ones this function owns.
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

  // Shoulder pivots sit ON the base-plate corner circle (each base-plate vertex
  // is the arm's base joint); the effector triangle is the inverted platform.
  g.t = g.base_radius;

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

// mm → screen pixel (side-view: x → horizontal, z → vertical-up). Uses its own
// scale so the whole arm fits; see kScaleSide.
static sf::Vector2f toScreenXZ(const sf::Vector3f & p, float cx, float cy) {
  return sf::Vector2f(cx + p.x * kScaleSide, cy - p.z * kScaleSide);
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

// Elbow (upper-arm end) from an arm angle (degrees).
// The upper arm sweeps downward out of the horizontal: outward component
// rf·cos(a) along the limb's radial, plus -rf·sin(a) in z.
static sf::Vector3f elbowFromAngle(const Geom & g, int limb, float armAngleDeg) {
  const float c = std::cos(armAngleDeg * kPi / 180.0f);
  const float r = g.upper_arm_len * c;
  const sf::Vector3f u = limbRadial(limb);
  return g.motors[limb] + sf::Vector3f(u.x * r, u.y * r, -g.upper_arm_len * std::sin(armAngleDeg * kPi / 180.0f));
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
static sf::Vector3f armUnitDir(int limb, float armAngleDeg) {
  const float c = std::cos(armAngleDeg * kPi / 180.0f);
  const float s = std::sin(armAngleDeg * kPi / 180.0f);
  const sf::Vector3f u = limbRadial(limb);
  return sf::Vector3f(u.x * c, u.y * c, -s);
}

// Unit PERPENDICULAR to the upper arm in the limb's vertical plane, pointing
// to the UNDERSIDE of the arm (so a=0 -> straight down -z): the direction in
// which the rod's arm-side ball joint stands off the arm axis.
static sf::Vector3f armStandoff(int limb, float armAngleDeg) {
  const float s = std::sin(armAngleDeg * kPi / 180.0f);
  const float c = std::cos(armAngleDeg * kPi / 180.0f);
  const sf::Vector3f u = limbRadial(limb);
  return sf::Vector3f(-u.x * s, -u.y * s, -c);
}

// Servo-linkage points: the "upper" rod is a crank of fixed length
// (upper_rod_len) keyed to the servo shaft s; the "lower" rod is a RIGID bar
// of constant length (servo_rod_len) from the crank pin e to the arm-side
// socket f. The socket stands a fixed perpendicular distance (arm_attach_offset)
// off the arm axis at a bracket that sits on the arm at arm_attach_dist from
// its base joint a.
//
// Measurement direction: the servo reading IS the horn angle, so the drawn
// crank is the measured input, not a quantity derived from the arm. The arm
// angle is then SOLVED from that crank by requiring the rigid rod to reach, so
// the chain runs servo -> horn -> arm, never arm -> horn.
// (The lower ARM, elbow -> platform, is a separate pair of parallel bars
// drawn in the limb loops.)
struct LinkPins
{
    sf::Vector3f e, f;
    sf::Vector3f arm;   // point ON the upper-arm axis where the attach bracket sits
};

// Horn mounting: the servo reading in degrees is the horn angle from the
// horizon, so kCrankZeroDeg is the reading that puts the horn along the
// outward horizontal and kCrankSign is which way the horn swings as the
// reading grows. These are the two numbers to change if the drawn horn turns
// the wrong way for the physical servo.
// Sign is -1 because the real arm's convention is POSITIVE DOWNWARD: a larger
// reading swings the horn BELOW the horizon, which is the sense the hardware
// turns as the arm drops.
static constexpr float kCrankZeroDeg = 0.0f;
static constexpr float kCrankSign    = -1.0f;

// Crank pin straight from the servo reading: at upper_rod_len from the shaft,
// at the reading's angle from the horizon in the limb's vertical plane.
static sf::Vector3f crankPin(int limb, float crank, float servoDeg,
                             const sf::Vector3f & s) {
  const sf::Vector3f u = limbRadial(limb);
  const float rs = s.x * u.x + s.y * u.y;
  const float a = (kCrankZeroDeg + kCrankSign * servoDeg) * kPi / 180.0f;
  return u * (rs + crank * std::cos(a)) +
         sf::Vector3f(0.0f, 0.0f, s.z + crank * std::sin(a));
}

// The rod's arm-side ball joint, standing arm_attach_offset off the arm axis.
static sf::Vector3f armSocket(const Geom & g, int limb, float armAngleDeg) {
  const sf::Vector3f d = armUnitDir(limb, armAngleDeg);
  return g.motors[limb] + d * g.arm_attach_dist +
         armStandoff(limb, armAngleDeg) * g.arm_attach_offset;
}

// Arm angle implied by a servo reading. The horn is rigid at the reading and
// the rod is rigid at servo_rod_len, so the socket must land exactly
// servo_rod_len from the horn pin. The socket rides a circle of radius
// hypot(arm_attach_dist, arm_attach_offset) about the arm's base joint, so
// that is a circle-vs-circle intersection with generally TWO assembly modes.
// `nearPhi` picks the branch by continuity, which is what stops the drawn arm
// from snapping between modes as the servo moves. False when the rod cannot
// reach the socket circle at any arm angle.
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
  // Horn from the measurement, rod length therefore implied by the solve.
  l.e = crankPin(limb, g.upper_rod_len, servoDeg, g.servos[limb]);
  return l;
}


// Forward kinematics from a set of upper-arm angles (degrees):
// intersect the three `lower_arm_len` spheres centered at (elbow - platform
// offset) to recover the end-effector centre. Returns the DOWNWARD root so the
// picture matches the real (hanging) arm. False if the three measured angles
// do not resolve to a valid pose.
static bool fkFromAngles(const Geom & g, const float angDeg[3], sf::Vector3f & e)
{
  sf::Vector3f c[3];
  for (int i = 0; i < 3; ++i) {
    const sf::Vector3f el = elbowFromAngle(g, i, angDeg[i]);
    c[i] = el - g.plat_off[i];
  }
  // A = c1 - c0, B = c2 - c0 (plane of the three sphere centres).
  const sf::Vector3f A = c[1] - c[0];
  const sf::Vector3f B = c[2] - c[0];
  const float gAA = A.x * A.x + A.y * A.y + A.z * A.z;
  const float gBB = B.x * B.x + B.y * B.y + B.z * B.z;
  const float gAB = A.x * B.x + A.y * B.y + A.z * B.z;
  const float denom = gAA * gBB - gAB * gAB;   // = |A x B|^2
  if (denom < 1e-6f) return false;             // degenerate: centres collinear

  // x (relative to c0) = p*A + q*B solves 2 A·x = gAA, 2 B·x = gBB (Cramer).
  const float p = (0.5f * gAA * gBB - 0.5f * gBB * gAB) / denom;
  const float q = (0.5f * gBB * gAA - 0.5f * gAA * gAB) / denom;
  const sf::Vector3f x0(A.x * p + B.x * q, A.y * p + B.y * q, A.z * p + B.z * q);

  // Remaining degree of freedom along n = A x B, chosen so |e - c0| = L.
  const float n2 = denom;
  const float disc = g.lower_arm_len * g.lower_arm_len - (x0.x * x0.x + x0.y * x0.y + x0.z * x0.z);
  if (disc < 0.0f) return false;               // measured pose unreachable
  const float lam = std::sqrt(disc / n2);
  const sf::Vector3f n(A.y * B.z - A.z * B.y, A.z * B.x - A.x * B.z, A.x * B.y - A.y * B.x);

  // Two mirror solutions about the centre-plane; keep the one that hangs DOWN.
  const sf::Vector3f lo = c[0] + x0 + n * lam;
  const sf::Vector3f hi = c[0] + x0 - n * lam;
  e = (lo.z < hi.z) ? lo : hi;
  return true;
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
static void drawLinkage(sf::RenderWindow & w, const Geom & g, int limb, float armAngleDeg,
                        float servoDeg,
                        sf::Vector2f (*proj)(const sf::Vector3f &, float, float),
                        float cx, float cy, bool outOfClosure = false) {
  const sf::Vector3f s = g.servos[limb];
  const LinkPins l = servoLinkage(g, limb, armAngleDeg, servoDeg);

  // Servo body: the servo is mounted UNDER the base plate, so its housing
  // hangs from the plate underside (z=0) down to the shaft plane.
  const sf::Vector3f postBase(s.x, s.y, 0.0f);
  drawLine(w, proj(postBase, cx, cy), proj(s, cx, cy), sf::Color(90, 90, 105), 5.0f);
  // UPPER rod: the single bar fixed to the servo shaft, pointing OUTWARD along
  // the arm direction, so the crank pin (and the rod's motor-side joint) sits
  // beyond the servo shaft's radius, away from the platform.
  // A limb whose measured servo angle the modelled 4-bar cannot close is drawn
  // in the warning colour: the linkage here is extrapolated, not a solved pose.
  const sf::Color upperCol = outOfClosure ? sf::Color(235, 120, 60) : sf::Color(150, 150, 165);
  const sf::Color lowerCol = outOfClosure ? sf::Color(255, 170, 90) : sf::Color(185, 185, 195);
  drawLine(w, proj(s, cx, cy), proj(l.e, cx, cy), upperCol, 3.0f);
  // LOWER rod: the single bar from the upper rod's far end to the arm's attach
  // bracket end (which stands off the arm axis by the attach offset).
  drawLine(w, proj(l.e, cx, cy), proj(l.f, cx, cy), lowerCol, 2.0f);
  // Standoff bracket: from the arm axis to the rod's arm-side socket end.
  drawLine(w, proj(l.arm, cx, cy), proj(l.f, cx, cy), upperCol, 1.5f);
  // Pins.
  drawDot(w, proj(l.e, cx, cy), 2.2f, sf::Color(90, 90, 100));
  drawDot(w, proj(l.f, cx, cy), 2.2f, sf::Color(90, 90, 100));
  if (outOfClosure) {
    // Ring the shaft so an extrapolated limb is obvious in either view.
    drawDot(w, proj(s, cx, cy), 5.0f, sf::Color(255, 90, 60));
  }
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DeltaArmSim::ArmSimSFMLNode>();
  node->enable_position_streaming();

  const Geom geom = loadGeometry(*node);
  const double step = node->declare_parameter<double>("step", 0.01);

  // Initial pose = the shared sim.initial_pos (also used by the controller).
  // The arm BEGINS at the IK solution for that pose instead of the flat/dead
  // {0,0,0} defaults, so the picture is valid before the first arm/pos msg and
  // the keys move relative to a start posture.
  const auto init_pos = node->declare_parameter<std::vector<double>>(
    "sim.initial_pos", std::vector<double>{0.0, 0.0, -0.30});
  node->declare_parameter<bool>("sim.simulate_arrival", true);
  double tx = init_pos.size() == 3 ? init_pos[0] : 0.0;
  double ty = init_pos.size() == 3 ? init_pos[1] : 0.0;
  double tz = init_pos.size() == 3 ? init_pos[2] : -0.30;

  // Seed the joint state from the IK of the initial pose, so the arm is drawn
  // posed from the first frame; live motor feedback still overrides it once the
  // driver reports. The IK runs through the shared DeltaArm::Arm, so the seeded
  // posture is guaranteed to be one the controller would also accept.
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

  // Frame clock: measures real time between rendered frames so the display
  // smoothing (update_display) is frame-rate independent.
  sf::Clock frame_clock;

  // Layout: left half = top view, right half = side view.
  const float halfW = 600.0f;
  const float cxTop  = halfW * 0.5f;
  const float cyTop  = 300.0f;
  const float cxSide = halfW + halfW * 0.5f;
  const float cySide = 176.0f;  // z=0 line: +120mm -> y=80, -340mm -> y=450

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

    // ── Divider + titles ────────────────────────────────────────────────
    drawLine(window, sf::Vector2f(halfW, 0), sf::Vector2f(halfW, 600),
             sf::Color(60, 60, 60), 1.0f);

    if (hasFont) {
      drawText(window, sf::Vector2f(10, 10), "Top View (XY)", font, 16,
               sf::Color(160, 160, 160));
      drawText(window, sf::Vector2f(halfW + 10, 10), "Side View (XZ)", font, 16,
               sf::Color(160, 160, 160));
      drawText(window, sf::Vector2f(10, 30),
               node->feedback_source_active()
                 ? "SOURCE: real servo feedback (arm/motor_feedback)"
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

    // ── Read arm state ──────────────────────────────────────────────────
    // When the real motor driver is publishing fresh, all-online feedback,
    // draw the REAL machine (measured servo angles + FK) instead of the
    // controller's sim stream (arm/pos). The source selection is sticky
    // (hysteresis + hold, see ArmSimSFMLNode) and the drawn angles are
    // smoothed, so a serial blip no longer shakes the picture.
    const bool live_fb = node->select_source();
    const float raw_ang[3] = {  // raw MOTOR angles from the selected source
      live_fb ? node->fb_ang_[0] : node->ang_[0],
      live_fb ? node->fb_ang_[1] : node->ang_[1],
      live_fb ? node->fb_ang_[2] : node->ang_[2]};
    node->update_display(raw_ang, frame_clock.restart().asSeconds());
    const float * ang = node->display_angles();  // smoothed, drawn (per limb)

    // MOTOR target angles (controller intent) + mapped ARM angles for both
    // current and target - shown in the legend so motor-space and arm-space
    // state are both visible. The motor -> arm conversion is the controller's
    // own solve (see ArmSimSFMLNode::arm_angle_deg), so what is drawn here is
    // the arm angle the controller's forward estimate is using too.
    const float ang_tar[3] = {
      node->ang_tar_[0], node->ang_tar_[1], node->ang_tar_[2]};
    const float armTar[3] = {
      node->arm_angle_deg(ang_tar[0], 0),
      node->arm_angle_deg(ang_tar[1], 1),
      node->arm_angle_deg(ang_tar[2], 2)};

    // Real-feedback rendering. The chain is servo -> horn -> arm, matching what
    // the user can measure on the hardware: the servo reading sets the horn
    // angle, and the arm angle is whatever the rigid rod forces. Deriving the
    // horn from the arm instead (the old direction) made the drawn crank a
    // function of the arm pose rather than of the measurement.
    // The horn pin fixes one end of the rod and the socket rides a circle, so
    // the solve has two assembly modes; `last_arm` is the continuity seed that
    // keeps the drawn arm on the branch the linkage is actually in. A reading
    // whose rod cannot reach at all is flagged rather than faked.
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
        // Rod cannot reach the socket at any arm angle: hold the last real pose
        // rather than inventing one.
        a = last_arm_valid[i] ? last_arm[i] : 60.0f;
      }
      armAng[i] = a;
    }

    sf::Vector3f e(static_cast<float>(node->pos_x_) * 1000.0f,  // m → mm
                   static_cast<float>(node->pos_y_) * 1000.0f,
                   static_cast<float>(node->pos_z_) * 1000.0f);
    // Real-feedback rendering only trusts the FK when every limb's linkage
    // actually closed: with a limb extrapolated past its closure limit the
    // three lower arms no longer describe a consistent platform, so keep the
    // last pose that did resolve instead of drawing a bogus end effector.
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

    // Controller's commanded target (arm/pos.target_position, m → mm): the
    // "control target point" the arm is driving toward.
    const sf::Vector3f tgt(static_cast<float>(node->tar_x_) * 1000.0f,
                           static_cast<float>(node->tar_y_) * 1000.0f,
                           static_cast<float>(node->tar_z_) * 1000.0f);
    const sf::Color tgtCol(255, 90, 220);

    sf::Vector3f elbow[3], plat[3];
    for (int i = 0; i < 3; ++i) {
      elbow[i] = elbowFromAngle(geom, i, armAng[i]);
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
      drawLinkage(window, geom, i, armAng[i], ang[i], toScreenXY, cxTop, cyTop, out_of_closure[i]);
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

    // Control target point + a faint line from the current EE (top view).
    drawLine(window, toScreenXY(e, cxTop, cyTop), toScreenXY(tgt, cxTop, cyTop),
             sf::Color(255, 90, 220, 110), 1.0f);
    drawDot(window, toScreenXY(tgt, cxTop, cyTop), 5.0f, sf::Color::Transparent, tgtCol);
    if (hasFont) drawLabel(window, toScreenXY(tgt, cxTop, cyTop), "TGT", font, 11, tgtCol);

    // ══════════════════════════════════════════════════════════════════════
    //  SIDE VIEW  (XZ plane – looking along Y, x → horizontal, z → up)
    // ══════════════════════════════════════════════════════════════════════

    // Ground line at z = 0.
    drawLine(window,
             sf::Vector2f(cxSide - geom.base_radius * kScaleSide, cySide),
             sf::Vector2f(cxSide + geom.base_radius * kScaleSide, cySide),
             sf::Color(60, 60, 60), 1.0f);

    // Servo linkage (side view): servo body below the ground line, single upper
    // rod on the shaft, single lower rod up to the arm attach point.
    for (int i = 0; i < 3; ++i) {
      drawLinkage(window, geom, i, armAng[i], ang[i], toScreenXZ, cxSide, cySide, out_of_closure[i]);
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

    // Control target point + faint line from the current EE (side view).
    drawLine(window, toScreenXZ(e, cxSide, cySide), toScreenXZ(tgt, cxSide, cySide),
             sf::Color(255, 90, 220, 110), 1.0f);
    drawDot(window, toScreenXZ(tgt, cxSide, cySide), 5.0f, sf::Color::Transparent, tgtCol);

    // ── Legend (right half, below the side view) ─────────────────────────
    // NOTE: the window is 1200x600, so every line must stay below y < 600;
    // the block is kept compact (14 px pitch) to make room for the angle
    // readouts.
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
      // Angle readouts: MOTOR angles (theta, servo convention) and ARM angles
      // (phi, mapped through the 4-bar linkage) - current vs TARGET (the last
      // set_pos goal mapped into the same space).
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