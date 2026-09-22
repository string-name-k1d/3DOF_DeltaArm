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
  // This is the SAME closed form the controller's ik_stage2 solves, so a motor
  // angle is mapped back to the arm angle before drawing (the arm is no longer
  // simply "rotated by the motor angle").
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
static Geom loadGeometry(rclcpp::Node & n)
{
  Geom g;
  g.base_radius = static_cast<float>(n.declare_parameter<double>("geometry.base_radius", 100.0));
  g.platform_radius = static_cast<float>(n.declare_parameter<double>("geometry.platform_radius", 32.5));
  g.upper_arm_len = static_cast<float>(n.declare_parameter<double>("geometry.upper_arm_len", 120.0));
  g.lower_arm_len = static_cast<float>(n.declare_parameter<double>("geometry.lower_arm_len", 240.0));

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
  g.servo_radius     = static_cast<float>(n.declare_parameter<double>("geometry.servo_radius", 57.65));
  g.servo_z          = static_cast<float>(n.declare_parameter<double>("geometry.servo_z", -22.5));
  g.upper_rod_len    = static_cast<float>(n.declare_parameter<double>("geometry.upper_rod_len", 60.0));
  g.servo_rod_len    = static_cast<float>(n.declare_parameter<double>("geometry.servo_rod_len", 35.0));
  g.arm_attach_dist  = static_cast<float>(n.declare_parameter<double>("geometry.arm_attach_dist", 68.5));
  g.arm_attach_offset = static_cast<float>(n.declare_parameter<double>("geometry.arm_attach_offset", 20.5));
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

// ── 4-bar motor <-> arm mapping (mirror of the controller's ik_stage2) ───────
// The servo drives the upper arm through a horn + rigid rod, so a MOTOR angle
// and the resulting ARM angle are related by the 4-bar closed form:
//   a = horn (upper_rod_len), b = rod (servo_rod_len),
//   c = shoulder->socket = hypot(arm_attach_dist, arm_attach_offset),
//   d = servo->shoulder = hypot(base_radius - servo_radius, -servo_z).
// The arm angle and motor angle are treated as two separate conventions now
// (arm: 0 = straight out, growing = downward; motor: folded into [0,2pi) with
// growing = arm sweeping down), exactly like the controller commands them.

// Smallest arm angle (deg) at which the linkage can still close (rod fully
// stretched): |socket(arm) - servo| == a + b.
static float bandLowDeg(const Geom & g) {
  const float a = g.upper_rod_len, b = g.servo_rod_len;
  float lo = 0.0f, hi = 90.0f;
  for (int i = 0; i < 48; ++i) {
    const float t = 0.5f * (lo + hi) * kPi / 180.0f;
    const float r = g.base_radius + g.arm_attach_dist * std::cos(t) - g.arm_attach_offset * std::sin(t);
    const float zz = -g.arm_attach_dist * std::sin(t) - g.arm_attach_offset * std::cos(t);
    const float dist = std::hypot(r - g.servo_radius, zz - g.servo_z);
    if (dist <= a + b) hi = 0.5f * (lo + hi); else lo = 0.5f * (lo + hi);
  }
  return 0.5f * (lo + hi);
}

// Motor angle (deg, driver convention) for a requested arm angle (deg).
// Clamped into the closing band so it never throws.
static float motorDegFromArm(const Geom & g, float armDeg) {
  const float lo = bandLowDeg(g);
  if (armDeg < lo) armDeg = lo;
  const float t = armDeg * kPi / 180.0f;
  const float a = g.upper_rod_len, b = g.servo_rod_len;
  const float cLink = std::hypot(g.arm_attach_dist, g.arm_attach_offset);
  const float dLink = std::hypot(g.base_radius - g.servo_radius, -g.servo_z);
  const float tb = kPi - t;                       // 180 deg - arm angle
  const float A = 2.0f * a * dLink * std::cos(tb) - 2.0f * b * dLink;
  const float B = 2.0f * a * dLink * std::sin(tb);
  const float C = cLink * cLink - a * a - b * b - dLink * dLink + 2.0f * a * b * std::cos(tb);
  const float R = std::hypot(A, B);
  const float u = (R > 1e-6f) ? C / R : 0.0f;
  const float raw = std::atan2(B, A) + std::acos(u < -1.0f ? -1.0f : (u > 1.0f ? 1.0f : u));
  return (kTwoPi - raw) * kRadToDeg;
}

// Arm angle (deg) whose closed-form motor angle equals motorDeg. Monotone
// bisection on the closing band; clamped to the band edges.
static float armDegFromMotor(const Geom & g, float motorDeg) {
  const float lo = bandLowDeg(g);
  const float hi = 90.0f;
  const float mAtLo = motorDegFromArm(g, lo);
  if (motorDeg <= mAtLo) return lo;
  const float mAtHi = motorDegFromArm(g, hi);
  if (motorDeg >= mAtHi) return hi;
  float a0 = lo, a1 = hi;
  for (int i = 0; i < 60; ++i) {
    const float mid = 0.5f * (a0 + a1);
    if (motorDegFromArm(g, mid) > motorDeg) a1 = mid; else a0 = mid;
  }
  return 0.5f * (a0 + a1);
}

// Servo-linkage points: the "upper" rod is a crank of fixed length
// (upper_rod_len) keyed to the servo shaft s; the "lower" rod is a RIGID bar
// of constant length (servo_rod_len) from the crank pin e to the arm-side
// socket f. The socket stands a fixed perpendicular distance (arm_attach_offset)
// off the arm axis at a bracket that sits on the arm at arm_attach_dist from
// its base joint a. The crank pin is solved so that |e - f| stays equal to
// servo_rod_len for every arm angle (true 4-bar), with the DOWNWARD branch
// chosen so the horn hangs below the shaft, i.e. toward the arm and platform
// (matches the physical servo-horn mounting).
// (The lower ARM, elbow -> platform, is a separate pair of parallel bars
// drawn in the limb loops.)
struct LinkPins
{
  sf::Vector3f e, f;
  sf::Vector3f arm;   // point ON the upper-arm axis where the attach bracket sits
};

// Crank pin: point at distance `crank` from the shaft s and at exactly
// `rodLen` from the attach f, in the limb's vertical plane (radius, z).
// The DOWNWARD (lower-z) intersection is preferred - the horn hangs below the
// shaft; if the circles do not intersect the pin is clamped toward f so the
// draw never explodes.
static sf::Vector3f crankPin(int limb, float crank, float rodLen,
                             const sf::Vector3f & s, const sf::Vector3f & f) {
  const sf::Vector3f u = limbRadial(limb);
  const float rs = s.x * u.x + s.y * u.y;
  const float rf = f.x * u.x + f.y * u.y;
  const float rx = rf - rs, rz = f.z - s.z;
  const float dist2 = rx * rx + rz * rz;
  const float dist = std::sqrt(dist2);
  if (dist < 1e-4f) return s + u * crank;
  const float along = (crank * crank - rodLen * rodLen + dist2) / (2.0f * dist);
  const float h2 = crank * crank - along * along;
  if (h2 < 0.0f) {  // unreachable at this arm angle: clamp the pin toward f
    const sf::Vector3f dir = f - s;
    const float len = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    return (len < 1e-4f) ? s + u * crank : s + dir * (crank / len);
  }
  const float h = std::sqrt(h2);
  const float ux = rx / dist, uz = rz / dist;
  const float qx = rs + along * ux, qz = s.z + along * uz;  // foot on s->f axis
  const float nx = -uz, nz = ux;                            // unit normal in-plane
  // Choose the circle-intersection branch that lies BELOW the shaft (lower
  // z), so the servo horn points down toward the arm. Fall back to the other
  // branch only if that one has an invalid (negative) horizontal radius.
  const float r1 = qx + h * nx, z1 = qz + h * nz;
  const float r2 = qx - h * nx, z2 = qz - h * nz;
  const bool use1 = (z1 <= z2) && (r1 > 0.0f);
  const float radius = use1 ? r1 : r2;
  const float zc = use1 ? z1 : z2;
  return u * (radius > 0.0f ? radius : 0.0f) + sf::Vector3f(0.0f, 0.0f, zc);
}

static LinkPins servoLinkage(const Geom & g, int limb, float armAngleDeg) {
  const sf::Vector3f s = g.servos[limb];
  const sf::Vector3f a = g.motors[limb];
  const sf::Vector3f d = armUnitDir(limb, armAngleDeg);
  LinkPins l;
  // Lower-rod attach point on the upper arm: fixed distance from the base joint.
  l.arm = a + d * g.arm_attach_dist;
  // The rod's arm-side ball joint stands OFF the arm axis by arm_attach_offset
  // (perpendicular, toward the arm underside) - the real bracket gap.
  l.f = l.arm + armStandoff(limb, armAngleDeg) * g.arm_attach_offset;
  // Crank pin keeps |e - f| = servo_rod_len (rigid rod) for any arm angle.
  l.e = crankPin(limb, g.upper_rod_len, g.servo_rod_len, s, l.f);
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

// Closed-form per-limb IK, mirroring the controller's delta_calc_angle_yz:
// target is the effector centre expressed in the limb frame where the outward
// radial is the NEGATIVE y axis and z is up. Returns the ARM angle in DEGREES
// (0 = limb straight out, growing angle sweeps the arm downward), choosing the
// smaller |angle| root. 0 is returned when the point is unreachable.
static float deltaLimbAngle(float x0, float y0, float z0,
                            float t, float pr, float rf, float re) {
  const float Y = t + y0 - pr;
  const float Z = z0;
  const float rho2 = Y * Y + Z * Z;
  if (rho2 < 1e-6f) return 0.0f;
  const float M = (re * re - x0 * x0 - rf * rf - rho2) / (2.0f * rf * std::sqrt(rho2));
  if (M < -1.0f || M > 1.0f) return 0.0f;   // circle miss: keep the limb flat
  const float phi = std::atan2(Z, Y);
  const float delta = std::acos(M);
  float th0 = phi + delta;
  const float th1 = phi - delta;
  if (std::fabs(th1) < std::fabs(th0)) th0 = th1;
  return th0 * kRadToDeg;
}

// IK: end-effector centre posMM (mm) -> the three ARM angles (deg) that put
// the arm there, using exactly the sim's own joint-frame convention, so the
// FK of the returned angles reproduces the requested pose.
static void ikFromPose(const Geom & g, const sf::Vector3f & posMM, float angDeg[3]) {
  const float x = posMM.x, y = posMM.y, z = posMM.z;
  for (int leg = 0; leg < 3; ++leg) {
    const float pa = leg * kTwoPiOver3;
    const float ca = std::cos(pa), sa = std::sin(pa);
    const float rx = x * ca + y * sa;
    const float ry = -x * sa + y * ca;
    angDeg[leg] = deltaLimbAngle(rx, ry, z, g.t, g.platform_radius,
                                 g.upper_arm_len, g.lower_arm_len);
  }
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
                        sf::Vector2f (*proj)(const sf::Vector3f &, float, float),
                        float cx, float cy) {
  const sf::Vector3f s = g.servos[limb];
  const LinkPins l = servoLinkage(g, limb, armAngleDeg);

  // Servo body: the servo is mounted UNDER the base plate, so its housing
  // hangs from the plate underside (z=0) down to the shaft plane.
  const sf::Vector3f postBase(s.x, s.y, 0.0f);
  drawLine(w, proj(postBase, cx, cy), proj(s, cx, cy), sf::Color(90, 90, 105), 5.0f);
  // UPPER rod: the single bar fixed to the servo shaft, pointing OUTWARD along
  // the arm direction, so the crank pin (and the rod's motor-side joint) sits
  // beyond the servo shaft's radius, away from the platform.
  drawLine(w, proj(s, cx, cy), proj(l.e, cx, cy), sf::Color(150, 150, 165), 3.0f);
  // LOWER rod: the single bar from the upper rod's far end to the arm's attach
  // bracket end (which stands off the arm axis by the attach offset).
  drawLine(w, proj(l.e, cx, cy), proj(l.f, cx, cy), sf::Color(185, 185, 195), 2.0f);
  // Standoff bracket: from the arm axis to the rod's arm-side socket end.
  drawLine(w, proj(l.arm, cx, cy), proj(l.f, cx, cy), sf::Color(150, 150, 165), 1.5f);
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

  // Seed the joint state from the IK of the initial pose (SIM geometry), so the
  // arm is drawn posed from the first frame; live motor feedback still overrides
  // it once the driver reports. ikFromPose returns ARM angles; convert them to
  // MOTOR angles first (the renderer maps motor -> arm every frame).
  {
    const sf::Vector3f initMM(static_cast<float>(tx) * 1000.0f,
                              static_cast<float>(ty) * 1000.0f,
                              static_cast<float>(tz) * 1000.0f);
    float init_ang[3];
    ikFromPose(geom, initMM, init_ang);
    node->ang_[0] = motorDegFromArm(geom, init_ang[0]);
    node->ang_[1] = motorDegFromArm(geom, init_ang[1]);
    node->ang_[2] = motorDegFromArm(geom, init_ang[2]);
    node->pos_x_ = tx;
    node->pos_y_ = ty;
    node->pos_z_ = tz;
  }

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
      drawText(window, sf::Vector2f(10, 30),
               node->feedback_live()
                 ? "SOURCE: real servo feedback (arm/motor_feedback)"
                 : "SOURCE: sim stream (arm/pos)",
               font, 13,
               node->feedback_live() ? sf::Color(120, 220, 120) : sf::Color(210, 200, 130));
    }

    // ── Read arm state ──────────────────────────────────────────────────
    // When the real motor driver is publishing fresh, all-online feedback,
    // draw the REAL machine (measured servo angles + FK) instead of the
    // controller's sim stream (arm/pos).
    const bool live_fb = node->feedback_live();
    const float ang[3] = {   // MOTOR angles (degrees, driver convention)
      live_fb ? node->fb_ang_[0] : node->ang_[0],
      live_fb ? node->fb_ang_[1] : node->ang_[1],
      live_fb ? node->fb_ang_[2] : node->ang_[2]};

    // Convert motor -> arm angle per limb through the 4-bar, then draw the
    // upper arm/linkage with the ARM angle (the mechanism no longer has the
    // arm simply rotating with the motor).
    const float armAng[3] = {
      armDegFromMotor(geom, ang[0]),
      armDegFromMotor(geom, ang[1]),
      armDegFromMotor(geom, ang[2])};

    sf::Vector3f e(static_cast<float>(node->pos_x_) * 1000.0f,  // m → mm
                   static_cast<float>(node->pos_y_) * 1000.0f,
                   static_cast<float>(node->pos_z_) * 1000.0f);
    if (live_fb) fkFromAngles(geom, armAng, e);

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
      drawLinkage(window, geom, i, armAng[i], toScreenXY, cxTop, cyTop);
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
      drawLinkage(window, geom, i, armAng[i], toScreenXZ, cxSide, cySide);
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