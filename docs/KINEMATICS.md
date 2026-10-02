# Delta-Arm Kinematics

This document describes the exact math implemented in `src/axis/delta-arm.cpp`
and mirrored in the SFML 2-D sim (`src/sim/arm_sim_sfml_main.cpp`), the 3-D
Gazebo arm (`arm_gazebo`), and the controller's parameter set. All lengths are
in **millimetres** internally (converted from the SI metres used by ROS
topics/actions), all angles stored in **radians**, and the motor-facing
outputs are folded to **degrees**.

There are three independent pieces:

| Piece | Function | Code |
|---|---|---|
| Stage-1 IK | task-space target → per-limb upper-arm angle | `Arm::ik_stage1()` / `delta_calc_angle_yz()` |
| Stage-2 IK | limb plane angle → **servo/motor** angle through the 4-bar | `Arm::ik_stage2()` → `Arm::motor_from_arm()` |
| Forward kinematics | servo angles → task-space position (feeds `cur_pos`) | `Arm::forward_kinematics()` → `Arm::arm_from_motor()` |

---

## 0. Conventions (locked, do not change independently)

- **Arm angle θ** (per-limb, rad): `0` = limb **horizontal** (straight out),
  *growing* = limb sweeps **downward**. The arm reaches its lowest point at
  `θ = 90°`, so the usable/driven range here is `[≈31.8°, 90°]` (see closure band).
- **Motor angle** (deg, reported by the driver / published on `motor_targets`):
  `0` maps to the limb horizontal only *formally*; the physically reachable
  motor interval is `[≈70.7°, ≈132.2°]` for `θ ∈ [≈31.8°, 90°]`. Motor angle
  grows as the arm sweeps down.
- Conversion: `motor_deg = (motor_rad + home_offset) · 180/π`, and the inverse
  uses `motor_rad = motor_deg · π/180 − home_offset`. `home_offset` is the
  calibration offset (default `0`).
- **Range fold:** the raw 4-bar root is folded with `motor = 2π − raw` so the
  motor angle is **monotone increasing** across the band and lands inside
  `[0, 2π)` (driver nominal range is `[0°, 145°]`).
- An out-of-band arm angle is **not** an error inside `motor_from_arm` — the
  stage-2 caller checks it against the resolved band (§3.1a, §6) and reports the
  goal as unreachable, leaving the previous targets committed.

---

## 1. Geometry (defaults)

Per-limb `ArmMechConfig` (`Arm::init_default_geometry()`); all three limbs are
equilateral (`plane_angle = leg · 120°`, leg ∈ {0,1,2}):

| Constant | Value | Meaning |
|---|---|---|
| `base_radius` | 100.0 | circumradius of the base triangle (shoulder circle), `t` |
| `platform_radius` | 32.5 | circumradius of the effector triangle, `pr` |
| `upper_arm_len` | 120.0 | upper rod (shoulder → elbow), `rf` |
| `lower_arm_len` | 240.0 | lower rod (elbow → platform joint), `re` |
| `servo_radius` | 57.65 | servo shaft under each plate corner, `sr` |
| `servo_z` | 22.5 | servo mount BELOW the shoulder plane, `sz` (sign handled in d) |
| `upper_rod_len` | 60.0 | 4-bar **link a** (servo horn) |
| `servo_rod_len` | 35.0 | 4-bar **link b** (connecting rod: horn → arm bracket socket) |
| `arm_attach_dist` | 68.5 | 4-bar link **c x**-component (shoulder → socket along the arm) |
| `arm_attach_offset` | 20.5 | 4-bar link **c z**-component (perpendicular standoff of the socket) |
| `home_offset` | 0.0 | motor calibration offset |

Derived 4-bar lengths:

```
c = hypot(arm_attach_dist, arm_attach_offset) = hypot(68.5, 20.5) ≈ 71.50
d = hypot(base_radius − servo_radius, servo_z) = hypot(42.35, 22.5) ≈ 47.96   (ground link)
```

---

## 2. Stage 1 — classic delta IK (`delta_calc_angle_yz`)

Rotate the target into the limb's local yz-plane then solve a circle
intersection per limb.

Given limb-local payload joint `V = (x0, y0−pr, z0)` and motor pivot
`A = (0, −t)` (t = `base_radius`), the elbow `E(b) = (0, −t − rf·cos b, −rf·sin b)`
must satisfy `|E − V| = re`:

```
(rf·cos b + Y)² + (rf·sin b + Z)² = re² − x0²          Y = t + y0 − pr,  Z = z0
 Y·cos b + Z·sin b = (re² − x0² − rf² − Y² − Z²) / (2 rf) = M · ρ   (ρ = |(Y,Z)|)
 b = φ ± acos(M/ρ),   φ = atan2(Z, Y)
```

Two roots exist; the code picks the one with the **smallest |b|** (the
outward-hanging arm). Throws on:
- `ρ ≤ 0` (target on the limb axis — on-axis singularity);
- `|M| > 1` (the two circles miss each other — unreachable radius).

The returned degree value becomes `stage1_thetas_[leg]` (radians) fed to stage 2.

---

## 3. Stage 2 — the 4-bar linkage (`motor_from_arm`)

The servo horn does **not** turn the arm directly; horn angle `θa` and arm
angle `θ` relate through a planar four-bar chain. Reference `arm` = arm link
(shoulder→socket, length `c`), `ground` = servo→shoulder (length `d`),
`input` = horn (length `a`), `coupler` = rod (length `b`).

Link topology (ground)
```
  shoulder(S) ──d── servo(O)          ground link d = |S−O|
     │                                   c-arm (shoulder→socket)
     c
    socket(P) ──b── horn_joint(J) ──a── servo(O)   (a = horn, b = rod)
```

With arm at angle `θ` (0 = horizontal), the arm-side socket sits at

```
P = ( base_radius + arm_attach_dist·cosθ − arm_attach_offset·sinθ ,
      −arm_attach_dist·sinθ − arm_attach_offset·cosθ )          (in z/down terms: sz)
```

### 3.1 Geometric closure test (before solving)

The rod physically spans from horn joint to socket only if

```
| |P − O| − (a+b) | ≤ tol   within   [ |a−b| , a+b ]
```

> `|a−b| = 25`, `a+b = 95`. At `θ = 0` the socket is ~110.9 mm from the servo —
> the arm CANNOT be held horizontal even though the algebraic solve below still
> returns a root (`u ≈ −0.68` is a legal `acos` argument). The band starts at
> the θ where `|P−O| = a+b`.

### 3.1a The resolved band (not a hard-coded constant)

The usable range is **not** a fixed `31.76°..90°`. It is derived once per
`set_geometry()` by `Arm::compute_linkage_bands()` as the intersection of:

1. **closure** — `four_bar_motor_angle()` is real (§3.1);
2. **monotonicity** — `motor_from_arm` must be non-decreasing in θ. Past the
   transmission-angle *fold* the map doubles back on itself and no single-valued
   inverse exists, so that region is unusable. The scan therefore takes the
   **longest strictly monotone sub-interval** of the closure run (this is what
   bounds the default assembly from below and gen0's lower edge);
3. **servo travel** — the commanded motor angle (after `home_offset`) must lie in
   `geometry.angle_min .. geometry.angle_max`;
4. **arm-angle window** — `geometry.arm_angle_min .. geometry.arm_angle_max`
   (default `0 .. 180°`).

```
default : [31.81°, 99.69°]   upper edge set by the 145° servo stop
gen0    : [ 0.05°, 101.13°]  upper edge set by the linkage
```

Relaxing the default servo stop to 200° opens the band to ~127°, i.e. **~3.4° of
arm per 5° of servo travel**. The arm does *not* stop at 90°; the servo does.

The band is logged at startup per leg and is the single authority for both
directions of the solve — see §6.

### 3.2 Closed-form solve

Set `θb = π − θ` (180° − arm angle — the angle the transport/parallelogram
mapping converts to). Standard Grashof crank-linkage triangle over `(a, b, c, d)`:

```
A = 2ad·cosθb − 2bd
B = 2ad·sinθb
C = c² − a² − b² − d² + 2ab·cosθb
R = |(A, B)| = hypot(A, B)          u = C / R        guard |u| ≤ 1 else throw
θa = atan2(B, A) ± acos(u)
```

The **`+`** acos branch is the physically-continuous one in the closing band
(the `−` branch produces a crossing/reflected configuration). The arm-convention
motor angle is the mirror of  `θa` about the servo-centre line, hence the range
fold:

```
motor_rad = 2π − θa⁺
```

### 3.3 Worked values (default geometry)

| Arm θ (deg) | Motor (deg) | Note |
|---|---|---|
| 31.76 (band_low) | ≈70.7 | rod fully stretched |
| 37.7 | 75.92 | live goal `(0,0,−0.25)` |
| 45 | ≈82.7 | |
| 48.7 | 86.19 | initial `(0,0,−0.28)` |
| 56.8 | 94.29 | live goal `(0,0,−0.30)` |
| 60 | ≈97.6 | |
| 75 | ≈114.1 | |
| 90 (down) | ≈132.2 | band_high |

`motor_from_arm` output (before `home_offset`): `deg` = `(motor_rad + home_offset)·180/π`.

---

## 4. Inverse 4-bar (`arm_from_motor`, used by FK)

`motor_from_arm` is monotone increasing across the **resolved band** (§3.1a), so
the inverse is a simple 60-iteration **bisection** on θ that re-evaluates the
closed form — never an algebraic inversion (the analytically swapped form fails
to replicate the `+` branch on part of `[85°, 128°]`-type regions).

```
θ = bisection over the resolved band [band_lo, band_hi] of  motor_from_arm(θ) == motor_rad
```

Motor angles outside the band are **clamped to the band edges**, so the function
is total: feedback from a servo that is not currently inside the resolved
envelope can never throw on the control path. This is what
`Arm::forward_kinematics()` uses to convert servo angles back into arm angles
before the delta FK, and it is why the drawn/streamed pose always agrees with a
goal the IK accepted.

---

## 5. Forward kinematics

1. Unpack each motor angle to `motor_rad = deg·π/180 − home_offset`.
2. `arm_from_motor` → limb angle `θi`.
3. Classic delta direct kinematics: sphere-triple intersection. The shoulder
   pivots sit ON the base circle (not lowered by the usual
   `(R−r)·tan30°/2`), radii `t` / `pr` as above; each rod constrains the
   effector centre to a sphere around the elbow offset by `pr` on the limb's
   radial. Solve the three planes → two candidate z, pick the downward one
   (the arm hangs below the plate).

Result: `cur_pos`/`cur_angles` used by `apply()`/`get_pos` and the live
`arm/pos` stream.

---

## 6. Reachability contract

`set_tar_pos()` returns `TargetResult{bool reached, std::string reason}` and is
**atomic**: on failure it commits no motor targets and does not advance the
commanded pose, so the arm holds position and the streamed target marker stays
where the arm actually is. Callers that only care about success may ignore it.

| condition | outcome |
|---|---|
| `ik_stage1` unreachable (no sphere intersection) | `reached = false`, reason `Unreachable delta target (circle miss)` |
| limb arm angle outside the resolved band (§3.1a) | `reached = false`, reason names the leg, the angle it needed and the band |
| any limb above the servo's travel | folded into the previous case — the band already accounts for it |
| stage 2 solves for all three limbs | `reached = true`, all three motor targets committed together |

The action server (`arm/set_pos`) maps this to `ABORTED` with the same reason and
the 2-D visualiser shows `TARGET REJECTED`, so a refused goal is visible rather
than looking like a dead motor.

`arm_from_motor` and `arm_angle_from_motor_deg` never throw and never leave the
band — they clamp to its edges. They are the *forward* direction and must be
total, because they run on feedback from real hardware.

---

## 7. Keeping everything in sync

The geometry/constants above are duplicated (by design, verified equal) in:

- `src/axis/delta-arm.cpp` — `Arm::init_default_geometry()`;
- `src/controller/delta_arm_controller_node.cpp` — `declareParams()`;
- `config/arm_params.yaml`;
- `gazebo/src/arm_cmd_bridge.cpp` — Gazebo 3-D cmd bridge.

The 2-D visualiser is deliberately **not** on this list: `ArmSimSFMLNode` owns a
`DeltaArm::Arm` and drives its poses through the shared `set_tar_pos()` /
`arm_angle_from_motor_deg()` / `get_linkage_band()`, so it can no longer disagree
with the controller about what is reachable. It still keeps its own 2-D drawing
helpers (`fkFromAngles`, the four-bar crank-pin position) because those describe
the picture, not the solution.

Change the rest together (a single `RMS`-verified unit test target covers the
outputs: `test/test_delta_arm.cpp` — 18/18 pass, run the binary directly,
`ctest` is broken in-container). Reference tests:

| Motor (deg) | expected arm (deg) |
|---|---|
| 75.92 | 37.68 |
| 86.19 | 48.66 |
| 94.29 | 56.78 |