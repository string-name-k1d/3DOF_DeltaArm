#include "arm/delta_arm.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace DeltaArm {

Motor::Motor() : Motor(0.0f) {}

Motor::Motor(float start_pos) : start_pos(start_pos), cur_pos(start_pos), tar_pos(start_pos), servo_id(0) {}

void Motor::set_tar_pos(float pos) { tar_pos = pos; }

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kCos120 = -0.5f;
constexpr float kSin120 = 0.8660254037844386f;
constexpr float kHalfPi = 1.57079632679489661923f;

// Stage 1: task-space point -> limb angle.
float delta_inverse_kinematics_arm(float x0, float y0, float z0,
                                   float phi_i, float A, float B, float C, float D,
                                   bool bypass = false) {


    const float p = x0 * std::cos(phi_i) + y0 * std::sin(phi_i);
    const float d2 = x0 * x0 + y0 * y0 + z0 * z0;


    const float P = (A - D) * (A - D) - 2.0f * (A - D) * p + d2 + B * B - C * C;
    const float Q = 2.0f * B * (A - D - p);
    const float R = 2.0f * B * z0;


    const float rho_sq = (A - D - p) * (A - D - p) + z0 * z0;
    if (rho_sq < 1e-6f) {
        throw std::invalid_argument("Unreachable delta target (on-axis singularity)");
    }

    float disc = -P * P + Q * Q + R * R;
    if (disc < 0.0f) {
        if (!bypass) {
            throw std::invalid_argument("Unreachable delta target (circle miss)");
        }


        disc = 0.0f;
    }


    const float root = std::sqrt(disc);
    const float denom = P - Q;
    float th0 = 2.0f * std::atan2(-R + root, denom);
    float th1 = 2.0f * std::atan2(-R - root, denom);


    auto fold = [](float a) {
        while (a > kPi) a -= kTwoPi;
        while (a <= -kPi) a += kTwoPi;
        return a;
    };
    th0 = fold(th0);
    th1 = fold(th1);


    const float th = (std::fabs(th0) <= std::fabs(th1)) ? th0 : th1;

    if (!std::isfinite(th)) {
        throw std::invalid_argument("Unreachable delta target (degenerate limb)");
    }
    return th;
}

// Stage 2: limb angle -> servo angle via exact 4-bar solve.
float four_bar_motor_angle(float theta_arm, const ArmMechConfig& c) {


    const float px = c.base_radius + c.arm_attach_dist * std::cos(theta_arm) -
                     c.arm_attach_offset * std::sin(theta_arm);
    const float py = -c.arm_attach_dist * std::sin(theta_arm) -
                     c.arm_attach_offset * std::cos(theta_arm);

    const float a = c.upper_rod_len;
    const float b = c.servo_rod_len;


    const float dx = px - c.servo_radius;
    const float dy = py - c.servo_z;
    const float d = std::hypot(dx, dy);


    if (d < 1e-6f || d > a + b + 1e-3f || d < std::fabs(a - b) - 1e-3f) {
        return std::numeric_limits<float>::quiet_NaN();
    }


    const float along = (d * d + a * a - b * b) / (2.0f * d);

    const float h = std::sqrt(std::max(0.0f, a * a - along * along));


    const float fx = c.servo_radius + along * (dx / d);
    const float fy = c.servo_z + along * (dy / d);


    const float hx = fx + h * (dy / d);
    const float hy = fy - h * (dx / d);


    float ang = std::atan2(c.servo_z - hy, hx - c.servo_radius);
    if (ang < 0.0f) ang += kTwoPi;
    return ang;
}

bool linkage_closes(float theta_arm, const ArmMechConfig& c) {
    return !std::isnan(four_bar_motor_angle(theta_arm, c));
}

constexpr float kBandScanStep = kPi / 9000.0f;

constexpr float kBandEpsilon = 0.05f * kDegToRad;

constexpr float kMotorMonotoneEps = 1e-6f;

}

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
    constexpr float kTwoPiOver3 = 2.0943951023931953f;
    configs_.clear();
    for (int leg = 0; leg < 3; ++leg) {
        ArmMechConfig c;
        c.base_radius = 100.0f;
        c.platform_radius = 32.5f;
        c.upper_arm_len = 120.0f;
        c.lower_arm_len = 240.0f;
        c.servo_radius = 57.65f;
        c.servo_z = -22.5f;
        c.upper_rod_len = 60.0f;
        c.servo_rod_len = 35.0f;
        c.arm_attach_dist = 68.5f;
        c.arm_attach_offset = 20.5f;
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

// Resolve the diagnostic arm-angle band from geometry and limits.
void Arm::compute_linkage_bands() {
    for (int leg = 0; leg < 3; ++leg) {
        const ArmMechConfig& c = configs_[leg];
        band_valid_[leg] = false;
        band_lo_[leg] = 0.0f;
        band_hi_[leg] = 0.0f;


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
        if (!found_run) continue;


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


                const float cmd_deg = (motor + c.home_offset) * kRadToDeg;
                ok = cmd_deg >= c.motor_angle_min - 1e-3f && cmd_deg <= c.motor_angle_max + 1e-3f;
            }
            if (ok) {
                ok = (th + kBandEpsilon) >= c.arm_angle_min && (th - kBandEpsilon) <= c.arm_angle_max;
            }
            if (ok && in_seg && (motor + kMotorMonotoneEps) < prev_motor) {
                ok = false;
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
        if (best_seg_len < 0.0f) continue;


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

// Stage 1 for all three limbs.
void Arm::ik_stage1(const Vec3& target) {

    const float x = target.x * 1000.0f;
    const float y = target.y * 1000.0f;
    const float z = target.z * 1000.0f;

    for (int leg = 0; leg < 3; ++leg) {
        const ArmMechConfig& c = configs_[leg];


        const float phi_i = c.plane_angle - kHalfPi;


        const bool bypass = c.bypass_reachability || c.raw_motor_clamp;

        stage1_thetas_[leg] = delta_inverse_kinematics_arm(
            x, y, z, phi_i, c.base_radius, c.upper_arm_len,
            c.lower_arm_len, c.platform_radius, bypass);
    }
}

float Arm::motor_from_arm(float theta_arm, int leg) const {
    const float motor = four_bar_motor_angle(theta_arm, configs_[leg]);
    if (std::isnan(motor)) {
        throw std::invalid_argument("four-bar linkage cannot close at this arm angle");
    }
    return motor;
}

float Arm::motor_from_arm_unchecked(float theta_arm, int leg) const {


    return four_bar_motor_angle(theta_arm, configs_[leg]);
}

float Arm::arm_from_motor(float motor_rad, int leg) const {
    if (configs_[leg].raw_motor_clamp) {


        const ArmMechConfig& c = configs_[leg];
        constexpr int kSteps = 1440;
        float best = c.arm_angle_min;
        float best_err = std::numeric_limits<float>::infinity();
        for (int i = 0; i <= kSteps; ++i) {
            const float t = c.arm_angle_min +
                (c.arm_angle_max - c.arm_angle_min) * (static_cast<float>(i) / kSteps);
            const float m = four_bar_motor_angle(t, c);
            if (std::isnan(m)) continue;
            const float err = std::fabs(m - motor_rad);
            if (err < best_err) {
                best_err = err;
                best = t;
            }
        }
        return best;
    }

    if (!band_valid_[leg]) {


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

// Stage 2 with raw-clamp or guarded behaviour.
float Arm::ik_stage2(float theta, int leg) {
    const ArmMechConfig& c = configs_[leg];


    if (c.raw_motor_clamp) {
        float arm_rad = theta;
        if (std::isnan(four_bar_motor_angle(arm_rad, c))) {
            arm_rad = nearest_closeable_arm_angle(arm_rad, leg);
            if (std::isnan(arm_rad)) {
                std::ostringstream os;
                os << "leg " << leg << " cannot close at ANY arm angle in ["
                   << (c.arm_angle_min * kRadToDeg) << ", "
                   << (c.arm_angle_max * kRadToDeg)
                   << "] deg; no motor command exists";
                throw std::invalid_argument(os.str());
            }
        }
        const float motor_rad = four_bar_motor_angle(arm_rad, c);
        return std::clamp((motor_rad + c.home_offset) * kRadToDeg,
                          c.motor_angle_min, c.motor_angle_max);
    }


    if (!band_valid_[leg] || theta < band_lo_[leg] || theta > band_hi_[leg]) {
        if (configs_[leg].bypass_reachability) {


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


    float angles[3];
    for (int leg = 0; leg < 3; ++leg) {
        angles[leg] = ik_stage2(stage1_thetas_[leg], leg);
    }
    for (int leg = 0; leg < 3; ++leg) {
        tar_angles_[leg] = angles[leg];
        motors_[leg].set_tar_pos(angles[leg]);
    }
}

// Measured motor angles -> end-effector estimate.
Vec3 Arm::forward_kinematics(const float angles_deg[3]) const {


    float ang_deg[3];
    for (int i = 0; i < 3; ++i) {
        const float motor_rad = angles_deg[i] * kDegToRad - configs_[i].home_offset;
        ang_deg[i] = arm_from_motor(motor_rad, i) * kRadToDeg;
    }


    const float re = configs_[0].lower_arm_len;


    const float t = configs_[0].base_radius;
    const float pr = configs_[0].platform_radius;


    const float k1 = kSin120;

    const float a1 = ang_deg[0] * kDegToRad;
    const float a2 = ang_deg[1] * kDegToRad;
    const float a3 = ang_deg[2] * kDegToRad;


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


    const float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

    const float w1 = y1 * y1 + z1 * z1;
    const float w2 = x2 * x2 + y2 * y2 + z2 * z2;
    const float w3 = x3 * x3 + y3 * y3 + z3 * z3;


    const float c1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
    const float d1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0f;


    const float c2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
    const float d2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0f;


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
    out.x = x0 * 0.001f;
    out.y = y0 * 0.001f;
    out.z = z0 * 0.001f;
    return out;
}

TargetResult Arm::set_tar_pos(float x, float y, float z) {
    const Vec3 goal{x, y, z};
    try {
        compute_ik(goal);
    } catch (const std::exception& e) {


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

void Arm::set_motor_targets(const float angles_deg[3]) {
    for (int i = 0; i < 3; ++i) {
        tar_angles_[i] = angles_deg[i];
        motors_[i].set_tar_pos(angles_deg[i]);
    }
    tar_pos_ = forward_kinematics(tar_angles_);
}

void Arm::stop() {

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


    for (int i = 0; i < 3; ++i) {
        cur_angles_[i] = motors_[i].tar_pos;
        motors_[i].cur_pos = cur_angles_[i];
    }

    cur_pos_ = forward_kinematics(cur_angles_);
}

void Arm::set_motor_feedback(const float angles_deg[3]) {


    for (int i = 0; i < 3; ++i) {
        cur_angles_[i] = angles_deg[i];
        motors_[i].cur_pos = angles_deg[i];
    }

    cur_pos_ = forward_kinematics(cur_angles_);
}

}
