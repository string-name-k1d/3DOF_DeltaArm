#include "arm/delta_arm.hpp"

#include <cmath>
#include <stdexcept>

namespace DeltaArm {

// ---------------------------------------------------------------------------
// Motor
// ---------------------------------------------------------------------------
Motor::Motor() : Motor(0.0f) {}

Motor::Motor(float start_pos) : start_pos(start_pos), cur_pos(start_pos), tar_pos(start_pos), servo_id(0) {}

void Motor::set_tar_pos(float pos) { tar_pos = pos; }

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kCos120 = -0.5f;
constexpr float kSin120 = 0.8660254037844386f; // sqrt(3)/2

/**
 * Solves the per-limb delta IK for the real mechanism:
 *
 *   * the three motor pivots lie on a circle of radius `t` about the base axis
 *     (equilateral, one pivot per limb plane);
 *   * the end-effector platform triangle has the SAME orientation as the base
 *     triangle, so the platform joint of a limb sits on the SAME radial line as
 *     its motor, at radius `pr` from the effector centre;
 *   * a limb of radius rf sweeps from the motor pivot, and the lower rod of
 *     length re closes from the upper-arm end (elbow) to the platform joint.
 *
 * `(x0, y0, z0)` is the end-effector centre in the limb's coordinate frame
 * (axis y = the limb's outward radial, z = up). Returns the motor angle in
 * DEGREES with the arm convention 0 deg = limb straight out, growing angle
 * sweeps the limb downward. Throws if the point is unreachable.
 *
 * Closed form: with A = (0,-t) the pivot, V = (x0, y0-pr, z0) the platform
 * joint and E(b) = (0, -t - rf*cos(b), -rf*sin(b)) the elbow:
 *
 *   (rf*cos b + Y)^2 + (rf*sin b + Z)^2 = re^2 - x0^2
 *     where  Y = t + y0 - pr,  Z = z0
 *   =>  Y*cos b + Z*sin b = (re^2 - x0^2 - rf^2 - Y^2 - Z^2) / (2*rf)  = M
 *   =>  b = phi +- acos(M/rho),  phi = atan2(Z, Y),  rho = |(Y,Z)|.
 *
 * Exactly one of the two roots corresponds to the outward-hanging arm; it is
 * the one with the smallest |b|.
 */
float delta_calc_angle_yz(float x0, float y0, float z0,
                          float t, float pr, float rf, float re) {
    const float Y = t + y0 - pr;
    const float Z = z0;
    const float rho_sq = Y * Y + Z * Z;
    if (rho_sq < 1e-6f) {
        throw std::invalid_argument("Unreachable delta target (on-axis singularity)");
    }

    const float M = (re * re - x0 * x0 - rf * rf - rho_sq) / (2.0f * rf * std::sqrt(rho_sq));
    if (M < -1.0f || M > 1.0f) {
        throw std::invalid_argument("Unreachable delta target (circle miss)");
    }

    const float phi = std::atan2(Z, Y);
    const float delta = std::acos(M);
    float th0 = phi + delta;
    float th1 = phi - delta;
    if (std::fabs(th1) < std::fabs(th0)) th0 = th1;

    return th0 * kRadToDeg;
}

} // namespace

// ---------------------------------------------------------------------------
// Arm
// ---------------------------------------------------------------------------
Arm::Arm(float start_angles[3]) {
    for (int i = 0; i < 3; ++i) {
        start_angles_[i] = (start_angles != nullptr) ? start_angles[i] : 0.0f;
        motors_[i] = Motor(start_angles_[i]);
        motors_[i].servo_id = i;
        cur_angles_[i] = start_angles_[i];
        tar_angles_[i] = start_angles_[i];
    }
    tar_pos_ = {0.0f, 0.0f, 0.0f};
    cur_pos_ = {0.0f, 0.0f, 0.0f};

    init_default_geometry();
}

Arm::~Arm() {}

void Arm::init() {
    for (int i = 0; i < 3; ++i) {
        start_angles_[i] = motors_[i].start_pos;
        motors_[i].cur_pos = start_angles_[i];
        motors_[i].tar_pos = start_angles_[i];
        cur_angles_[i] = start_angles_[i];
        tar_angles_[i] = start_angles_[i];
    }
    cur_pos_ = {0.0f, 0.0f, 0.0f};
    tar_pos_ = {0.0f, 0.0f, 0.0f};
}

void Arm::init_default_geometry() {
    constexpr float kTwoPiOver3 = 2.0943951023931953f; // 120 deg
    configs_.clear();
    for (int leg = 0; leg < 3; ++leg) {
        ArmMechConfig c;
        c.base_radius = 150.0f;    // mm (circumradius of the base triangle)
        c.platform_radius = 60.0f; // mm (circumradius of the effector triangle)
        c.upper_arm_len = 160.0f;  // mm
        c.lower_arm_len = 200.0f;  // mm
        c.plane_angle = leg * kTwoPiOver3;
        c.home_offset = 0.0f;
        configs_.push_back(c);
    }
}

void Arm::set_geometry(const std::vector<ArmMechConfig>& configs) {
    if (configs.size() != 3) {
        throw std::invalid_argument("set_geometry requires exactly 3 limb configs");
    }
    configs_ = configs;
}

// ---------------------------------------------------------------------------
// Two-stage inverse kinematics (classic 3-DOF delta)
// ---------------------------------------------------------------------------

/**
 * STAGE 1 - task-space target -> per-limb plane orientation.
 *
 * For each limb, rotate the 3D target into that limb's local frame (so the
 * limb lies in its yz-plane) and solve the classic delta IK. The resulting
 * upper-arm angle (stored in stage1_thetas_[]) is the per-limb orientation
 * angle that stage 2 maps to a motor angle.
 */
void Arm::ik_stage1(const Vec3& target) {
    // Convert metres -> millimetres (the geometry uses mm).
    const float x = target.x * 1000.0f;
    const float y = target.y * 1000.0f;
    const float z = target.z * 1000.0f;

    // Shoulder-pivot radius: the arm's base joints sit ON the base-plate
    // corner circle (base_radius). The effector joints sit at platform_radius,
    // which is INBOARD of the shoulders (classic delta). Same radii/pivots as
    // the forward kinematics and the visualizer.
    const float t = configs_[0].base_radius;

    for (int leg = 0; leg < 3; ++leg) {
        // Rotate the target into the limb's frame (the limb's plane becomes the
        // yz-plane). The angle for leg 0 is 0; legs 1 & 2 at +-120 deg.
        const ArmMechConfig& c = configs_[leg];
        const float ca = std::cos(c.plane_angle);
        const float sa = std::sin(c.plane_angle);
        const float rx = x * ca + y * sa;
        const float ry = -x * sa + y * ca;

        // Upper-arm angle in degrees for this limb (real-mechanism solver).
        const float theta_deg = delta_calc_angle_yz(rx, ry, z, t, c.platform_radius, c.upper_arm_len, c.lower_arm_len);
        stage1_thetas_[leg] = theta_deg * kDegToRad;
    }
}

/**
 * STAGE 2 - limb plane orientation -> motor joint angle (degrees).
 *
 * Applies the limb's home/calibration offset and converts to the driver's
 * degrees convention.
 */
float Arm::ik_stage2(float theta, int leg) { return (theta * kRadToDeg + configs_[leg].home_offset * kRadToDeg); }

void Arm::compute_ik(const Vec3& target) {
    ik_stage1(target);
    for (int leg = 0; leg < 3; ++leg) {
        tar_angles_[leg] = ik_stage2(stage1_thetas_[leg], leg);
        motors_[leg].set_tar_pos(tar_angles_[leg]);
    }
}

/**
 * Forward kinematics: given the three upper-arm angles (degrees), compute the
 * end-effector position (metres) by intersecting the three spheres of radius
 * `lower_arm_len` centered on the three arm ends (classic delta direct
 * kinematics, DeltaKin reference).
 */
Vec3 Arm::forward_kinematics(const float angles_deg[3]) const {
    // Geometry (same radii/pivots as the IK): the shoulder pivots sit on the
    // base-plate corner circle in this machine (NOT lowered by (R-r)*tan30/2).
    const float re = configs_[0].lower_arm_len;

    // Shoulder-pivot radius (as above) and effector triangle radius.
    const float t = configs_[0].base_radius;
    const float pr = configs_[0].platform_radius;

    // Limb radial directions (equilateral, one per limb plane). The elbow of a
    // limb sits on its radial at (t + rf*cos(a)) with z = -rf*sin(a).
    const float k1 = kSin120; // leg 1 radial: (sin120, +0.5); leg 0: (0,-1); leg 2: (-sin120,+0.5)

    const float a1 = angles_deg[0] * kDegToRad;
    const float a2 = angles_deg[1] * kDegToRad;
    const float a3 = angles_deg[2] * kDegToRad;

    // Sphere centres for the rod constraints: elbow minus its platform-joint
    // offset (the platform joint of a limb lies on the same radial as its
    // motor, i.e. the effector triangle is apex-down). Each rod of length re
    // then constrains the effector CENTRE to a sphere of radius re.
    const float u1 = t + configs_[0].upper_arm_len * std::cos(a1) - pr;
    const float y1 = -u1;
    const float z1 = -configs_[0].upper_arm_len * std::sin(a1);

    const float u2 = t + configs_[1].upper_arm_len * std::cos(a2) - pr;
    const float x2 = u2 * k1;
    const float y2 = u2 * 0.5f;
    const float z2 = -configs_[1].upper_arm_len * std::sin(a2);

    const float u3 = t + configs_[2].upper_arm_len * std::cos(a3) - pr;
    const float x3 = -u3 * kSin120;
    const float y3 = u3 * 0.5f;
    const float z3 = -configs_[2].upper_arm_len * std::sin(a3);

    // Denominator for the linear solve.
    const float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

    const float w1 = y1 * y1 + z1 * z1;
    const float w2 = x2 * x2 + y2 * y2 + z2 * z2;
    const float w3 = x3 * x3 + y3 * y3 + z3 * z3;

    // x = (a1*z + b1)/dnm
    const float c1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
    const float d1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0f;

    // y = (a2*z + b2)/dnm
    const float c2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
    const float d2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0f;

    // a*z^2 + b*z + cc = 0
    const float aa = c1 * c1 + c2 * c2 + dnm * dnm;
    const float bb = 2.0f * (c1 * d1 + c2 * (d2 - y1 * dnm) - z1 * dnm * dnm);
    const float cc = (d2 - y1 * dnm) * (d2 - y1 * dnm) + d1 * d1 + dnm * dnm * (z1 * z1 - re * re);

    const float disc = bb * bb - 4.0f * aa * cc;
    if (disc < 0.0f) {
        return {0.0f, 0.0f, 0.0f};
    }

    const float z0 = -0.5f * (bb + std::sqrt(disc)) / aa;
    const float x0 = (c1 * z0 + d1) / dnm;
    const float y0 = (c2 * z0 + d2) / dnm;

    Vec3 out;
    out.x = x0 * 0.001f; // mm -> m
    out.y = y0 * 0.001f;
    out.z = z0 * 0.001f;
    return out;
}

// ---------------------------------------------------------------------------
// Public state interface
// ---------------------------------------------------------------------------
void Arm::set_tar_pos(float x, float y, float z) {
    tar_pos_ = {x, y, z};
    try {
        compute_ik(tar_pos_);
    } catch (const std::exception&) {
        // Unreachable target: leave the previous targets intact.
    }
}

Vec3 Arm::get_tar_pos() const { return tar_pos_; }

Vec3 Arm::get_cur_pos() const { return cur_pos_; }

void Arm::get_motor_current(float out[3]) const {
    for (int i = 0; i < 3; ++i)
        out[i] = cur_angles_[i];
}

void Arm::get_motor_targets(float out[3]) const {
    for (int i = 0; i < 3; ++i)
        out[i] = motors_[i].tar_pos;
}

void Arm::stop() {
    // Emergency halt: cancel any commanded motion and zero the motor outputs.
    for (int i = 0; i < 3; ++i) {
        motors_[i].set_tar_pos(0.0f);
        motors_[i].cur_pos = 0.0f;
        cur_angles_[i] = 0.0f;
        tar_angles_[i] = 0.0f;
    }
    tar_pos_ = {0.0f, 0.0f, 0.0f};
    cur_pos_ = {0.0f, 0.0f, 0.0f};
}

void Arm::apply() {
    // Promote the commanded targets to the "current" state and refresh the
    // forward-position estimate.
    for (int i = 0; i < 3; ++i) {
        cur_angles_[i] = motors_[i].tar_pos;
        motors_[i].cur_pos = cur_angles_[i];
    }

    cur_pos_ = forward_kinematics(cur_angles_);
}

} // namespace DeltaArm
