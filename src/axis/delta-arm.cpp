#include "arm/delta_arm.hpp"

#include <cmath>
#include <limits>
#include <sstream>
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
constexpr float kTwoPi = 6.28318530717958647692f;
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

/**
 * Non-throwing 4-bar closed form: arm angle (rad) -> motor angle (rad), folded
 * into [0, 2*pi) with the driver's convention (0 deg = limb fully extended,
 * growing angle = arm sweeping downward).
 *
 *   a = horn (upper_rod_len)   b = rod (servo_rod_len)
 *   c = shoulder -> socket     d = servo -> shoulder (ground)
 *   theta_b = pi - arm_angle
 *   A = 2 a d cos(theta_b) - 2 b d
 *   B = 2 a d sin(theta_b)
 *   C = c^2 - a^2 - b^2 - d^2 + 2 a b cos(theta_b)
 *   motor = 2*pi - (atan2(B, A) + acos(C / |(A,B)|))        (+ home offset, deg)
 *
 * Returns NaN when the linkage CANNOT close, which takes two independent
 * conditions: the rigid rod must span the servo shaft and the arm-side socket,
 * AND the horn solve must have a real acos argument. Both are needed - rod
 * reach alone admits angles where the algebra has no solution (short horn with
 * a long rod). This is the single shared predicate for both directions of the
 * solve, so the IK and the forward kinematics can never disagree about which
 * arm angles are reachable.
 */
float four_bar_motor_angle(float theta_arm, const ArmMechConfig& c) {
    const float a = c.upper_rod_len;
    const float b = c.servo_rod_len;

    const float sr = c.base_radius + c.arm_attach_dist * std::cos(theta_arm) -
                     c.arm_attach_offset * std::sin(theta_arm);
    const float sz = -c.arm_attach_dist * std::sin(theta_arm) -
                     c.arm_attach_offset * std::cos(theta_arm);
    const float dist = std::hypot(sr - c.servo_radius, sz - c.servo_z);
    if (dist > a + b + 1e-3f || dist < std::fabs(a - b) - 1e-3f) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    const float cc = std::hypot(c.arm_attach_dist, c.arm_attach_offset);
    const float dd = std::hypot(c.base_radius - c.servo_radius, -c.servo_z);
    const float tb = kPi - theta_arm;
    const float A = 2.0f * a * dd * std::cos(tb) - 2.0f * b * dd;
    const float B = 2.0f * a * dd * std::sin(tb);
    const float C = cc * cc - a * a - b * b - dd * dd + 2.0f * a * b * std::cos(tb);
    const float u = C / std::hypot(A, B);
    if (u < -1.0f || u > 1.0f) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    return kTwoPi - (std::atan2(B, A) + std::acos(u));
}

/**
 * True when the servo 4-bar linkage can physically close at this arm angle
 * (radians). See four_bar_motor_angle() for the closure conditions.
 */
bool linkage_closes(float theta_arm, const ArmMechConfig& c) {
    return !std::isnan(four_bar_motor_angle(theta_arm, c));
}

/// Resolution of the reachable-band scan, in radians (~0.02 deg).
constexpr float kBandScanStep = kPi / 9000.0f;
/// The resolved band is pulled this far inside the true envelope so that every
/// arm angle the band admits is strictly inside the closure region. Shrinking
/// inward is the safe direction: it can only make the IK refuse a pose, never
/// let it accept one that the forward solve would then have to clamp.
constexpr float kBandEpsilon = 0.05f * kDegToRad;
/// Tolerance when testing that the motor angle is non-decreasing across the
/// band (guards the bisection in arm_from_motor()).
constexpr float kMotorMonotoneEps = 1e-6f;

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
        c.base_radius = 100.0f;     // mm (circumradius of the base triangle)
        c.platform_radius = 32.5f;  // mm (circumradius of the effector triangle)
        c.upper_arm_len = 120.0f;   // mm
        c.lower_arm_len = 240.0f;   // mm
        c.servo_radius = 57.65f;    // mm (servo shafts under the plate corners)
        c.servo_z = -22.5f;         // mm (mount below the shoulder plane)
        c.upper_rod_len = 60.0f;    // mm (servo horn = 4-bar link a)
        c.servo_rod_len = 35.0f;    // mm (connecting rod = 4-bar link b)
        c.arm_attach_dist = 68.5f;  // mm (shoulder -> arm bracket, link c)
        c.arm_attach_offset = 20.5f;// mm (perpendicular bracket standoff)
        c.plane_angle = leg * kTwoPiOver3;
        c.home_offset = 0.0f;
        configs_.push_back(c);
    }
    compute_linkage_bands();
}

void Arm::set_geometry(const std::vector<ArmMechConfig>& configs) {
    if (configs.size() != 3) {
        throw std::invalid_argument("set_geometry requires exactly 3 limb configs");
    }
    configs_ = configs;
    compute_linkage_bands();
}

/**
 * Resolve the reachable upper-arm angle band of every limb from the current
 * geometry. This is the authoritative definition of "can the arm be there",
 * and BOTH directions of the 4-bar solve are driven from it, so the IK can only
 * accept poses the forward kinematics reproduces exactly.
 *
 * The band is the intersection of four constraints:
 *   1. the linkage can physically close (four_bar_motor_angle() is real);
 *   2. the motor angle is non-decreasing in the arm angle - past the four-bar's
 *      transmission-angle fold the map folds back on itself and no
 *      single-valued inverse exists, so that region is not usable;
 *   3. the commanded motor angle is inside the servo's mechanical travel;
 *   4. the arm angle is inside the configured arm_angle_min/arm_angle_max.
 *
 * Constraint 1 alone is not the answer: the closure region wraps around the
 * turn (the linkage "closes" again near +-180 deg) and, for the default
 * geometry, extends well past 90 deg - the arm does not stop sweeping up just
 * because it is perpendicular to the base. The widest closure run is picked to
 * resolve the wrap, then 2-4 trim it down.
 *
 * Runs once per set_geometry(); both solves are hot paths, so the band is
 * cached rather than searched per call.
 */
void Arm::compute_linkage_bands() {
    for (int leg = 0; leg < 3; ++leg) {
        const ArmMechConfig& c = configs_[leg];
        band_valid_[leg] = false;
        band_lo_[leg] = 0.0f;
        band_hi_[leg] = 0.0f;

        // 1. Every maximal run of closing arm angles over a full turn.
        float run_lo = 0.0f;
        float best_lo = 0.0f;
        float best_hi = 0.0f;
        bool in_run = false;
        bool found_run = false;
        const int n = static_cast<int>(kTwoPi / kBandScanStep);
        for (int i = 0; i <= n; ++i) {
            const float th = -kPi + static_cast<float>(i) * kBandScanStep;
            const bool closes = linkage_closes(th, c);
            if (closes && !in_run) {
                run_lo = th;
                in_run = true;
            }
            if (!closes && in_run) {
                in_run = false;
                if (!found_run || (th - run_lo) > (best_hi - best_lo)) {
                    best_lo = run_lo;
                    best_hi = th;
                    found_run = true;
                }
            }
        }
        if (in_run && (!found_run || (kPi - run_lo) > (best_hi - best_lo))) {
            best_lo = run_lo;
            best_hi = kPi;
            found_run = true;
        }
        if (!found_run) continue;  // the four-bar can never close on this limb

        // 2. Longest sub-interval of that run satisfying 2-4 above. A segment is
        //    finalised when it ends so that its OWN endpoints are the ones
        //    compared - tracking the length separately from the endpoints would
        //    otherwise keep the last (possibly tiny) run but its longest
        //    neighbour's length.
        float best_seg_lo = 0.0f;
        float best_seg_hi = 0.0f;
        float best_seg_len = -1.0f;
        float cur_lo = 0.0f;
        float cur_hi = 0.0f;
        bool in_seg = false;
        float prev_motor = 0.0f;
        const int steps = static_cast<int>((best_hi - best_lo) / kBandScanStep);
        for (int i = 0; i <= steps; ++i) {
            const float th = best_lo + static_cast<float>(i) * kBandScanStep;
            const float motor = four_bar_motor_angle(th, c);

            bool ok = !std::isnan(motor);
            if (ok) {
                // Servo travel is checked on the COMMANDED angle, i.e. after the
                // home offset, because that is what the driver clamps.
                const float cmd_deg = (motor + c.home_offset) * kRadToDeg;
                ok = cmd_deg >= c.motor_angle_min - 1e-3f && cmd_deg <= c.motor_angle_max + 1e-3f;
            }
            if (ok) {
                ok = (th + kBandEpsilon) >= c.arm_angle_min && (th - kBandEpsilon) <= c.arm_angle_max;
            }
            if (ok && in_seg && (motor + kMotorMonotoneEps) < prev_motor) {
                ok = false;  // four-bar fold: no single-valued inverse past here
            }

            if (ok) {
                if (!in_seg) {
                    cur_lo = th;
                    in_seg = true;
                }
                cur_hi = th;
            } else if (in_seg) {
                in_seg = false;
                if ((cur_hi - cur_lo) > best_seg_len) {
                    best_seg_len = cur_hi - cur_lo;
                    best_seg_lo = cur_lo;
                    best_seg_hi = cur_hi;
                }
            }
            if (ok) prev_motor = motor;
        }
        if (in_seg && (cur_hi - cur_lo) > best_seg_len) {
            best_seg_len = cur_hi - cur_lo;
            best_seg_lo = cur_lo;
            best_seg_hi = cur_hi;
        }
        if (best_seg_len < 0.0f) continue;  // nothing survives the servo / arm-angle window

        // Inward by an epsilon so the advertised edges are strictly inside the
        // region the scan actually validated.
        band_lo_[leg] = best_seg_lo + kBandEpsilon;
        band_hi_[leg] = best_seg_hi - kBandEpsilon;
        band_valid_[leg] = band_hi_[leg] > band_lo_[leg];
        if (!band_valid_[leg]) {
            band_lo_[leg] = 0.0f;
            band_hi_[leg] = 0.0f;
        }
    }
}

bool Arm::get_linkage_band(int leg, float& lo_deg, float& hi_deg) const {
    if (leg < 0 || leg > 2) return false;
    lo_deg = band_lo_[leg] * kRadToDeg;
    hi_deg = band_hi_[leg] * kRadToDeg;
    return band_valid_[leg];
}

float Arm::arm_angle_from_motor_deg(float motor_deg, int leg) const {
    const float motor_rad = motor_deg * kDegToRad - configs_[leg].home_offset;
    return arm_from_motor(motor_rad, leg) * kRadToDeg;
}

bool Arm::linkage_motor_span(int leg, float & motor_lo_deg, float & motor_hi_deg) const {
    if (!band_valid_[leg]) return false;
    motor_lo_deg = motor_from_arm_unchecked(band_lo_[leg], leg) * kRadToDeg;
    motor_hi_deg = motor_from_arm_unchecked(band_hi_[leg], leg) * kRadToDeg;
    return true;
}

bool Arm::try_arm_from_motor_deg(float motor_deg, int leg, float & arm_out) const {
    if (!band_valid_[leg]) return false;

    const float motor_rad = motor_deg * kDegToRad - configs_[leg].home_offset;
    const float m_lo = motor_from_arm_unchecked(band_lo_[leg], leg);
    const float m_hi = motor_from_arm_unchecked(band_hi_[leg], leg);
    // Outside the closure range there is no arm angle that reproduces this
    // motor angle on the modelled linkage: report it instead of clamping.
    if (motor_rad <= m_lo || motor_rad >= m_hi) return false;

    float lo = band_lo_[leg];
    float hi = band_hi_[leg];
    for (int i = 0; i < 60; ++i) {
        const float mid = 0.5f * (lo + hi);
        if (motor_from_arm_unchecked(mid, leg) > motor_rad) hi = mid; else lo = mid;
    }
    arm_out = 0.5f * (lo + hi) * kRadToDeg;
    return true;
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
 * The servo does NOT turn the arm directly: it drives a 4-bar linkage, so the
 * limb angle and motor angle are NON-linear functions of one another. With
 *   a = horn (upper_rod_len)     b = rod (servo_rod_len)
 *   c = shoulder->socket         d = servo->shoulder (ground)
 * and  theta_b = 180 deg - (arm angle), the motor-side horn angle solves
 *   A = 2 a d cos(theta_b) - 2 b d
 *   B = 2 a d sin(theta_b)
 *   C = c^2 - a^2 - b^2 - d^2 + 2 a b cos(theta_b)
 *   theta_a = atan2(B, A) +- acos( C / |(A,B)| )      (closed form)
 * The arm-side link is length `c`, so c = hypot(arm_attach_dist,
 * arm_attach_offset) and the ground link is d = servo->shoulder length.
 * The "+" acos branch is the physically-continuous one in the closing band.
 *
 * The result is folded into the range [0, 2pi) with the convention 0 deg =
 * limb fully extended and GROWING angle sweeping the arm DOWNWARD (fits the
 * driver's nominal [0,145] deg range), then the home/calibration offset is
 * added. The algebra lives in four_bar_motor_angle(); this wrapper only turns
 * "the linkage cannot close" into an exception so the IK has something to
 * report. Callers should normally have checked the limb's reachable band
 * first (see compute_linkage_bands()), which is tighter than closure alone.
 */
float Arm::motor_from_arm(float theta_arm, int leg) const {
    const float motor = four_bar_motor_angle(theta_arm, configs_[leg]);
    if (std::isnan(motor)) {
        throw std::invalid_argument("four-bar linkage cannot close at this arm angle");
    }
    return motor;
}

float Arm::motor_from_arm_unchecked(float theta_arm, int leg) const {
    // Only ever called on arm angles that compute_linkage_bands() already
    // proved close, so the result is always a real number. Nothing here can
    // throw, which is what keeps the forward estimate exception-free.
    return four_bar_motor_angle(theta_arm, configs_[leg]);
}

/**
 * INVERSE 4-bar solve: motor angle (rad) -> arm angle (rad).
 *
 * Bisects the limb's reachable band, which compute_linkage_bands() guarantees
 * is a region where motor_from_arm() is real and non-decreasing, so the
 * bisection is well posed. Motor angles outside the band - the servo sitting at
 * a hard stop, or feedback from a pose the current geometry cannot represent -
 * are clamped to the nearest band edge.
 *
 * Never throws: this runs inside apply() on every control tick, and an
 * exception escaping the controller's timer callback would take the node down.
 * It is the forward estimate, so clamping is the right behaviour - the arm
 * physically cannot be anywhere else.
 */
float Arm::arm_from_motor(float motor_rad, int leg) const {
    if (!band_valid_[leg]) {
        // No reachable arm angle on this limb. The IK refuses every pose, so
        // the only job here is to stay finite and deterministic.
        return 0.0f;
    }

    const float m_lo = motor_from_arm_unchecked(band_lo_[leg], leg);
    const float m_hi = motor_from_arm_unchecked(band_hi_[leg], leg);
    if (motor_rad <= m_lo) return band_lo_[leg];
    if (motor_rad >= m_hi) return band_hi_[leg];

    float lo = band_lo_[leg];
    float hi = band_hi_[leg];
    for (int i = 0; i < 60; ++i) {
        const float mid = 0.5f * (lo + hi);
        if (motor_from_arm_unchecked(mid, leg) > motor_rad) hi = mid; else lo = mid;
    }
    return 0.5f * (lo + hi);
}

float Arm::ik_stage2(float theta, int leg) {
    // The band check is the reachable test. motor_from_arm() would only reject
    // arm angles outside the linkage's closure region; the band is tighter (it
    // also folds in the servo travel and the arm-angle window), so a pose the
    // driver could not actually hold is refused here instead of being commanded
    // and then silently clamped by the servo.
    if (!band_valid_[leg] || theta < band_lo_[leg] || theta > band_hi_[leg]) {
        std::ostringstream os;
        os << "leg " << leg << " needs arm angle " << (theta * kRadToDeg) << " deg, outside the reachable band ";
        if (band_valid_[leg]) {
            os << "[" << (band_lo_[leg] * kRadToDeg) << ", " << (band_hi_[leg] * kRadToDeg) << "] deg";
        } else {
            os << "(empty: the linkage cannot close within the servo's travel)";
        }
        throw std::invalid_argument(os.str());
    }

    const float motor_rad = motor_from_arm(theta, leg);
    return (motor_rad + configs_[leg].home_offset) * kRadToDeg;
}

void Arm::compute_ik(const Vec3& target) {
    ik_stage1(target);
    // Solve every limb before committing any of them: a target can become
    // unreachable on leg 2 after legs 0 and 1 have succeeded, and a partial
    // commit would leave the arm half-way to a pose that was rejected.
    float angles[3];
    for (int leg = 0; leg < 3; ++leg) {
        angles[leg] = ik_stage2(stage1_thetas_[leg], leg);
    }
    for (int leg = 0; leg < 3; ++leg) {
        tar_angles_[leg] = angles[leg];
        motors_[leg].set_tar_pos(angles[leg]);
    }
}

/**
 * Forward kinematics: given the three upper-arm angles (degrees), compute the
 * end-effector position (metres) by intersecting the three spheres of radius
 * `lower_arm_len` centered on the three arm ends (classic delta direct
 * kinematics, DeltaKin reference).
 */
Vec3 Arm::forward_kinematics(const float angles_deg[3]) const {
    // Convert the SERVO (motor) angles back to the limb upper-arm angles
    // through the 4-bar linkage, then run the classic delta FK on the arm
    // angles. arm_from_motor() clamps to the limb's reachable band and cannot
    // throw, so a servo resting at a hard stop yields the nearest pose the
    // mechanism can actually be in rather than killing the control loop.
    float ang_deg[3];
    for (int i = 0; i < 3; ++i) {
        const float motor_rad = angles_deg[i] * kDegToRad - configs_[i].home_offset;
        ang_deg[i] = arm_from_motor(motor_rad, i) * kRadToDeg;
    }

    // Geometry (same radii/pivots as the IK): the shoulder pivots sit on the
    // base-plate corner circle in this machine (NOT lowered by (R-r)*tan30/2).
    const float re = configs_[0].lower_arm_len;

    // Shoulder-pivot radius (as above) and effector triangle radius.
    const float t = configs_[0].base_radius;
    const float pr = configs_[0].platform_radius;

    // Limb radial directions (equilateral, one per limb plane). The elbow of a
    // limb sits on its radial at (t + rf*cos(a)) with z = -rf*sin(a).
    const float k1 = kSin120; // leg 1 radial: (sin120, +0.5); leg 0: (0,-1); leg 2: (-sin120,+0.5)

    const float a1 = ang_deg[0] * kDegToRad;
    const float a2 = ang_deg[1] * kDegToRad;
    const float a3 = ang_deg[2] * kDegToRad;

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
TargetResult Arm::set_tar_pos(float x, float y, float z) {
    const Vec3 goal{x, y, z};
    try {
        compute_ik(goal);
    } catch (const std::exception& e) {
        // Unreachable target: leave the previous motor targets and the previous
        // commanded position intact, but report why. Swallowing this silently
        // made a rejected goal look exactly like a dead motor.
        return TargetResult{false, e.what()};
    }
    tar_pos_ = goal;
    return TargetResult{true, {}};
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
