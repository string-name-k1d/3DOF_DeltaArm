#include "arm/delta_arm.hpp"

#include <algorithm>
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
constexpr float kHalfPi = 1.57079632679489661923f;

/**
 * Inverse kinematics for ONE limb of a 3-DOF delta arm, transcribed step by step
 * from
 *
 *   R. L. Williams II, "The Delta Parallel Robot: Kinematics Solutions",
 *   Ohio University (linked from arm/README.md).
 *
 * The paper's notation is kept verbatim so the code reads against it:
 *
 *   D_vec = (x0, y0, z0)  end-effector (platform) centre, world frame
 *   e_i    = (cos phi_i, sin phi_i, 0)   unit vector along limb i's radial
 *   e_z    = (0, 0, 1)
 *   A      = |A_i|              origin      -> shoulder       (base_radius)
 *   B      = |A_i - B_i|        shoulder    -> elbow          (upper_arm_len)
 *   C      = |B_i - C_i|        elbow       -> platform joint (lower_arm_len)
 *   D      = |D_vec - C_i|      origin      -> platform joint (platform_radius)
 *
 * Positions, exactly as the paper writes them:
 *   A_i = A e_i                                     shoulder
 *   C_i = D_vec + D e_i                             platform joint
 *   B_i = A_i + B (e_i cos(theta_i) - e_z sin(theta_i))   elbow
 *
 * so the arm angle theta_i runs from 0 (limb straight out radially) and grows as
 * the elbow descends, matching this codebase's sign convention.
 *
 * The shoulder and the platform joint both lie in the plane spanned by
 * (e_i, e_z), so only the radial and vertical components of D_vec enter the
 * closure equation - see step 4 for how the perpendicular one is folded in.
 *
 * @param x0,y0,z0  target centre in millimetres.
 * @param phi_i     world angle of limb i's radial; see ik_stage1 for the -90 deg
 *                  offset this codebase applies to the paper's 2*pi*i/3.
 * @param A,B,C,D   the four link lengths in millimetres.
 * @param bypass    when true an out-of-workspace target is clamped to the
 *                  nearest tangent pose instead of throwing.
 *
 * @return theta_i in RADIANS.
 * @throws std::invalid_argument when the target is unreachable.
 */
float delta_inverse_kinematics_arm(float x0, float y0, float z0,
                                   float phi_i, float A, float B, float C, float D,
                                   bool bypass = false) {
    // Step 1: project the target onto limb i's radial axis.  In the rotated
    // frame e_i is the +x axis, so this is simply p = D_vec . e_i, and
    //   |D_vec|^2 = p^2 + q^2 + z0^2
    // with q the perpendicular component of the target.
    const float p = x0 * std::cos(phi_i) + y0 * std::sin(phi_i);
    const float d2 = x0 * x0 + y0 * y0 + z0 * z0;  // |D_vec|^2

    // Step 2: the platform joint C_i = D_vec + D e_i lies at radial (p + D),
    //         height z0, and off-plane by q.
    // Step 3: the elbow B_i lies at radial (A + B cos theta_i), height
    //         (-B sin theta_i), and on-plane.
    //
    // Step 4: impose the rod constraint |B_i - C_i| = C and collect theta_i:
    //
    //   (A + B cos(theta_i) - p - D)^2 + (B sin(theta_i) + z0)^2 + q^2 = C^2
    //
    // Reducing with cos^2 + sin^2 = 1, and replacing (D + p)^2 + q^2 by
    // d^2 + 2 D p + D^2, gives the linear equation
    //
    //     P + Q cos(theta_i) + R sin(theta_i) = 0
    const float P = (A - D) * (A - D) - 2.0f * (A - D) * p + d2 + B * B - C * C;
    const float Q = 2.0f * B * (A - D - p);
    const float R = 2.0f * B * z0;

    // Step 5: the workspace test.  Writing (Q/2B, R/2B) = rho (cos psi,
    // sin psi), the equation is rho cos(theta_i - psi) = -P/2, hence
    //   -P^2 + Q^2 + R^2 = 4 B^2 rho^2 (1 - cos^2(theta_i - psi)) >= 0.
    // A negative value means no real elbow exists: the limb cannot span the
    // target at all.  The shoulder sitting on the platform joint is separately
    // degenerate, because then the rod triangle and the arm angle collapse.
    const float rho_sq = (A - D - p) * (A - D - p) + z0 * z0;
    if (rho_sq < 1e-6f) {
        throw std::invalid_argument("Unreachable delta target (on-axis singularity)");
    }

    float disc = -P * P + Q * Q + R * R;
    if (disc < 0.0f) {
        if (!bypass) {
            throw std::invalid_argument("Unreachable delta target (circle miss)");
        }
        // Diagnostic mode: clamp to the tangent pose, the closest point of this
        // limb's reachable surface.  Note the tangent the clamp lands on is
        // still selected by P's sign through the Q and R above, so over- and
        // under-reach clamp to the two different tangencies.
        disc = 0.0f;
    }

    // Step 6: the half-angle substitution t = tan(theta_i / 2) turns the
    // equation into the quadratic
    //
    //     (P - Q) t^2 + 2 R t + (P + Q) = 0
    //
    // whose two roots give the assembly modes.  Undoing the substitution with
    // atan2 rather than the paper's atan keeps the result correct in every
    // quadrant, including the degenerate case P == Q.
    const float root = std::sqrt(disc);
    const float denom = P - Q;
    float th0 = 2.0f * std::atan2(-R + root, denom);
    float th1 = 2.0f * std::atan2(-R - root, denom);

    // Both modes come out in (-2*pi, 2*pi]; fold each into (-pi, pi].
    auto fold = [](float a) {
        while (a > kPi) a -= kTwoPi;
        while (a <= -kPi) a += kTwoPi;
        return a;
    };
    th0 = fold(th0);
    th1 = fold(th1);

    // Step 7: assembly mode.  The paper resolves each limb independently by
    // taking the root closest to the home (fully lowered) position, i.e. the
    // smaller |theta_i|, which folds up instead of inverting through the top as
    // the target crosses the axis.  Hard-coded rather than configurable.
    const float th = (std::fabs(th0) <= std::fabs(th1)) ? th0 : th1;

    if (!std::isfinite(th)) {
        throw std::invalid_argument("Unreachable delta target (degenerate limb)");
    }
    return th;
}

/**
 * Exact 4-bar solve for one limb, done entirely in that limb's own (radial, z)
 * plane:
 *
 *   O = (servo_radius, servo_z)   servo output shaft, i.e. the horn pivot
 *   S = (base_radius, 0)          shoulder pivot
 *   P(theta_arm)                  socket on the upper arm, |P - S| = cc
 *
 *   a = |E - O| = upper_rod_len   horn  (crank)
 *   b = |E - P| = servo_rod_len   rod   (coupler)
 *
 * The horn pin E is therefore the intersection of circle(O, a) with circle(P, b),
 * which is what is solved below.  This replaces an earlier atan2/acos closed
 * form, which did not describe a rigid horn: sweeping the arm in small steps and
 * differencing its output gave crank increments drifting from about +3 deg to
 * -13 deg instead of staying at 0, so the arm could not be driven smoothly.
 *
 * Returns the servo angle in RADIANS folded into [0, 2*pi) with the driver's
 * convention - 0 deg = horn pointing straight out from the servo, growing angle =
 * horn sweeping DOWNWARD, which is what fits the nominal [0, 145] deg travel.
 *
 * Returns NaN when the linkage cannot close, i.e. when the rod is too short or
 * too long to span the shaft and the socket: |a - b| <= |P - O| <= a + b.  That is
 * the complete closure condition for this 4-bar and it is the single shared
 * predicate for both directions of the solve, so the IK and the forward
 * kinematics can never disagree about which arm angles are reachable.
 */
float four_bar_motor_angle(float theta_arm, const ArmMechConfig& c) {
    // Socket on the upper arm. The two terms rotate the arm-side offset with the
    // arm angle, and hypot(arm_attach_dist, arm_attach_offset) is the constant
    // shoulder -> socket length, so |P - S| = cc at every arm angle.
    const float px = c.base_radius + c.arm_attach_dist * std::cos(theta_arm) -
                     c.arm_attach_offset * std::sin(theta_arm);
    const float py = -c.arm_attach_dist * std::sin(theta_arm) -
                     c.arm_attach_offset * std::cos(theta_arm);

    const float a = c.upper_rod_len;
    const float b = c.servo_rod_len;

    // Vector from the servo shaft to the socket, and its length d.
    const float dx = px - c.servo_radius;
    const float dy = py - c.servo_z;
    const float d = std::hypot(dx, dy);

    // Closure: the two links must be able to meet at a point on the line O -> P.
    if (d < 1e-6f || d > a + b + 1e-3f || d < std::fabs(a - b) - 1e-3f) {
        return std::numeric_limits<float>::quiet_NaN();
    }

    // Circle-circle intersection. Along the O -> P line the horn pin sits
    // `along` from O, and `h` off the line. `along` is the projection of the
    // a-length leg onto d, and h is the altitude of the triangle with sides
    // a, b, d.
    const float along = (d * d + a * a - b * b) / (2.0f * d);
    // max() only absorbs the tolerance slop of the closure test above.
    const float h = std::sqrt(std::max(0.0f, a * a - along * along));

    // Foot of the perpendicular from E onto the O -> P line, then the pin itself.
    const float fx = c.servo_radius + along * (dx / d);
    const float fy = c.servo_z + along * (dy / d);

    // Assembly mode. The machine is built with the horn pin on the CLOCKWISE
    // side of the directed line O -> P: that is the branch whose servo angle
    // rises monotonically as the arm sweeps downward, while the other pin is the
    // fold-through-the-top branch. The two pins coincide only when h == 0, a
    // toggle of the crank where the pin flips over, and the band scan already
    // rejects those, so this rule is continuous over every valid band.
    const float hx = fx + h * (dy / d);
    const float hy = fy - h * (dx / d);

    // Servo angle of the horn, measured from the horizon and growing downward.
    float ang = std::atan2(c.servo_z - hy, hx - c.servo_radius);
    if (ang < 0.0f) ang += kTwoPi;
    return ang;
}

/**
 * True when the servo 4-bar linkage can physically close at this arm angle
 * (radians). See four_bar_motor_angle() for the closure condition.
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
 * For each limb, solve the classic delta IK of Williams II, "The Delta Parallel
 * Robot: Kinematics Solutions" (see arm/README.md), transcribed step by step in
 * delta_inverse_kinematics_arm() above. The resulting upper-arm angle (stored in
 * stage1_thetas_[], RADIANS) is the per-limb orientation angle that stage 2 maps
 * to a motor angle.
 */

void Arm::ik_stage1(const Vec3& target) {
    // Convert metres -> millimetres (the geometry uses mm).
    const float x = target.x * 1000.0f;
    const float y = target.y * 1000.0f;
    const float z = target.z * 1000.0f;

    for (int leg = 0; leg < 3; ++leg) {
        const ArmMechConfig& c = configs_[leg];
        // The paper sets phi_i = 2*pi*i/3, which would put shoulder 0 on +X.
        // This codebase numbers the motors with limb 0 toward -Y instead (see
        // ArmMechConfig::plane_angle and the renderer), so every radial carries a
        // constant -90 deg offset from the paper's. That single constant is the
        // only difference between the two conventions.
        const float phi_i = c.plane_angle - kHalfPi;
        // Per-limb upper-arm angle in RADIANS.
        stage1_thetas_[leg] = delta_inverse_kinematics_arm(
            x, y, z, phi_i, c.base_radius, c.upper_arm_len,
            c.lower_arm_len, c.platform_radius, c.bypass_reachability);
    }
}

/**
 * STAGE 2 - limb plane orientation -> motor joint angle (degrees).
 *
 * The servo does NOT turn the arm directly: it drives a 4-bar linkage, so the
 * limb angle and motor angle are NON-linear functions of one another. The solve
 * is geometric and exact - the horn pin is the intersection of the circle of
 * radius `a` (horn, upper_rod_len) about the servo shaft with the circle of
 * radius `b` (rod, servo_rod_len) about the arm-side socket. See
 * four_bar_motor_angle() above for the derivation and for how the assembly mode
 * is chosen.
 *
 * The result is folded into the range [0, 2pi) with the convention 0 deg = horn
 * straight out and GROWING angle sweeping the arm DOWNWARD (fits the driver's
 * nominal [0,145] deg range), then the home/calibration offset is added. The
 * geometry lives in four_bar_motor_angle(); this wrapper only turns "the linkage
 * cannot close" into an exception so the IK has something to report. Callers
 * should normally have checked the limb's reachable band first (see
 * compute_linkage_bands()), which is tighter than closure alone.
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

float Arm::nearest_closeable_arm_angle(float theta_arm, int leg) const {
    const ArmMechConfig& c = configs_[leg];
    if (!std::isnan(four_bar_motor_angle(theta_arm, c))) return theta_arm;

    // Scan the configured arm window for the closest angle that closes. The
    // window is only a few hundred degrees wide, so a dense sweep is cheap and
    // sidesteps the branch structure entirely.
    const float lo = c.arm_angle_min;
    const float hi = c.arm_angle_max;
    constexpr int kSteps = 1440;
    float best = std::numeric_limits<float>::quiet_NaN();
    float best_d = std::numeric_limits<float>::infinity();
    for (int i = 0; i <= kSteps; ++i) {
        const float t = lo + (hi - lo) * (static_cast<float>(i) / static_cast<float>(kSteps));
        if (std::isnan(four_bar_motor_angle(t, c))) continue;
        const float d = std::fabs(t - theta_arm);
        if (d < best_d) {
            best_d = d;
            best = t;
        }
    }
    return best;
}

float Arm::ik_stage2(float theta, int leg) {
    // The band check is the reachable test. motor_from_arm() would only reject
    // arm angles outside the linkage's closure region; the band is tighter (it
    // also folds in the servo travel and the arm-angle window), so a pose the
    // driver could not actually hold is refused here instead of being commanded
    // and then silently clamped by the servo.
    if (!band_valid_[leg] || theta < band_lo_[leg] || theta > band_hi_[leg]) {
        if (configs_[leg].bypass_reachability) {
            // Diagnostic mode: do not refuse. Fall back to the nearest arm angle
            // at which the linkage closes at all, so the run continues and the
            // resulting pose can be compared against the real mechanism.
            const float t = nearest_closeable_arm_angle(theta, leg);
            if (std::isnan(t)) {
                std::ostringstream os;
                os << "leg " << leg << " cannot close at ANY arm angle in ["
                   << (configs_[leg].arm_angle_min * kRadToDeg) << ", "
                   << (configs_[leg].arm_angle_max * kRadToDeg)
                   << "] deg with bypass_reachability on";
                throw std::invalid_argument(os.str());
            }
            theta = t;
        } else {
            std::ostringstream os;
            os << "leg " << leg << " needs arm angle " << (theta * kRadToDeg) << " deg, outside the reachable band ";
            if (band_valid_[leg]) {
                os << "[" << (band_lo_[leg] * kRadToDeg) << ", " << (band_hi_[leg] * kRadToDeg) << "] deg";
            } else {
                os << "(empty: the linkage cannot close within the servo's travel)";
            }
            throw std::invalid_argument(os.str());
        }
    }

    const float motor_rad = motor_from_arm(theta, leg);
    float cmd_deg = (motor_rad + configs_[leg].home_offset) * kRadToDeg;

    // Bypass mode deliberately resolves out-of-envelope targets to an arm angle
    // that merely CLOSES the linkage, ignoring the servo-travel term of the
    // band. The matching motor angle can therefore sit far outside the servo's
    // travel (gen0 gives 230 deg, or 356 deg for a shallow goal), i.e. a servo
    // angle the mechanism cannot hold and the driver would only clamp silently.
    //
    // Clamp the COMMANDED angle - the same post-home_offset quantity
    // compute_linkage_bands() tests and the driver limits - into the
    // intersection of the driver's travel and the band's closure span. Both
    // bounds are needed: the travel alone still permits angles below the
    // linkage's low closure limit (gen0 clamps to 6 deg, which does not close),
    // while the closure span alone already implies the travel. The result is that
    // bypass can never publish an unphysical or unresolvable servo target, and
    // forward kinematics agrees with the command instead of quietly re-clamping.
    // No-op for in-band poses, which satisfy both bounds by construction.
    if (configs_[leg].bypass_reachability) {
        float lo = configs_[leg].motor_angle_min;
        float hi = configs_[leg].motor_angle_max;
        if (band_valid_[leg]) {
            lo = std::max(lo, (motor_from_arm_unchecked(band_lo_[leg], leg) +
                               configs_[leg].home_offset) * kRadToDeg);
            hi = std::min(hi, (motor_from_arm_unchecked(band_hi_[leg], leg) +
                               configs_[leg].home_offset) * kRadToDeg);
        }
        cmd_deg = std::clamp(cmd_deg, lo, hi);
    }
    return cmd_deg;
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
