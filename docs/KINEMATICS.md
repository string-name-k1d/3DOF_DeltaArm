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
| Stage-1 IK | task-space target → per-limb upper-arm angle | `Arm::ik_stage1()` / `delta_inverse_kinematics_arm()` |
| Stage-2 IK | limb plane angle → **servo/motor** angle through the 4-bar | `Arm::ik_stage2()` → `Arm::motor_from_arm()` |
| Forward kinematics | servo angles → task-space position (feeds `cur_pos`) | `Arm::forward_kinematics()` → `Arm::arm_from_motor()` |

---

## 0. Conventions (locked, do not change independently)

- **Arm angle θ** (per-limb, rad): `0` = limb **horizontal** (straight out),
  *growing* = limb sweeps **downward**. The arm reaches its lowest point at
  `θ = 90°`, so the usable/driven range is the resolved band (§3.1a): on default
  `[≈31.8°, ≈124.8°]`, on gen0 `[≈0°, ≈78.2°]`.
- **Motor angle** (deg, reported by the driver / published on `motor_targets`):
  `0` is the horn pointing straight out from the servo; *growing* = horn sweeps
  **downward**. The physically reachable motor interval follows the band — on
  default `[≈20.2°, ≈133.1°]`, on gen0 `[≈15.9°, ≈144.9°]` (driver nominal range
  is `[0°, 145°]`).
- Conversion: `motor_deg = (motor_rad + home_offset) · 180/π`, and the inverse
  uses `motor_rad = motor_deg · π/180 − home_offset`. `home_offset` is the
  calibration offset (default `0`).
- **Range fold:** the horn pin's `atan2` result is folded into `[0, 2π)` so the
  motor angle is **monotone increasing** across the band and lands inside the
  driver's nominal range.
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

## 2. Stage 1 — classic delta IK (`delta_inverse_kinematics_arm`)

Transcribed step by step from

> R. L. **Williams II**, *The Delta Parallel Robot: Kinematics Solutions*,
> Ohio University — <https://people.ohio.edu/williams/html/PDF/DeltaKin.pdf>
> (linked from `arm/README.md`).

The paper's notation is kept verbatim in the code so it can be read against the
PDF. Per limb `i`:

| symbol | meaning | this codebase |
|---|---|---|
| `D_vec` | end-effector centre, world frame | the IK target, mm |
| `e_i` | `(cos φ_i, sin φ_i, 0)`, limb `i`'s radial | — |
| `e_z` | `(0,0,1)` | — |
| `A` | origin → shoulder | `base_radius` |
| `B` | shoulder → elbow | `upper_arm_len` |
| `C` | elbow → platform joint | `lower_arm_len` |
| `D` | origin → platform joint | `platform_radius` |
| `φ_i` | world angle of limb `i`'s radial | `plane_angle − 90°` |

```
A_i = A·e_i                                            shoulder
C_i = D_vec + D·e_i                                    platform joint
B_i = A_i + B (e_i cos θ_i − e_z sin θ_i)              elbow
```

`θ_i` runs from 0 (limb straight out radially) and grows as the elbow descends,
matching the driver's sign convention.

**Convention note.** The paper sets `φ_i = 2πi/3`, which would place shoulder 0 on
`+X`. This codebase numbers the motors with limb 0 toward `−Y` (see
`ArmMechConfig::plane_angle` and the renderer), so every radial carries a constant
`−90°` offset. That single constant is the only difference between the two
conventions.

### 2.1 Reduce the rod constraint to a linear equation in `θ_i`

Project the target onto the radial, `p = D_vec · e_i`, and note that
`|D_vec|² = p² + q² + z0²` with `q` the perpendicular component. The shoulder and
the platform joint both lie in the plane spanned by `(e_i, e_z)`, so `q` is
θ-independent and folds into the constant term:

```
(A + B cos θ_i − p − D)² + (B sin θ_i + z0)² + q² = C²
  ⇒  P + Q cos θ_i + R sin θ_i = 0

P = (A−D)² − 2(A−D)p + |D_vec|² + B² − C²
Q = 2B(A − D − p)
R = 2Bz0
```

### 2.2 Workspace test

Writing `(Q/2B, R/2B) = ρ(cos ψ, sin ψ)`, the equation is `ρ cos(θ_i − ψ) = −P/2`,
hence the discriminant

```
disc = −P² + Q² + R² = 4B²ρ² (1 − cos²(θ_i − ψ)) ≥ 0
```

Throws on:

- `ρ ≤ 0` — the platform joint lands on the shoulder (on-axis singularity), which
  collapses the rod triangle;
- `disc < 0` — no real elbow exists; the limb cannot span the target at all
  (circle miss).

Under the `bypass_reachability` diagnostic switch `disc` is clamped to `0`, giving
the tangent pose — the closest reachable point. The tangent reached still depends
on `P`'s sign through `Q` and `R`, so over- and under-reach clamp to the two
different tangencies.

### 2.3 Half-angle solve

`t = tan(θ_i/2)` linearises the equation into a quadratic:

```
(P − Q)t² + 2Rt + (P + Q) = 0
  ⇒  θ_i  = 2·atan2(−R + √disc, P − Q)
      θ_i' = 2·atan2(−R − √disc, P − Q)        both folded into (−π, π]
```

`atan2` is used in place of the paper's `atan` so the result is correct in every
quadrant and also when `P = Q`.

**Assembly mode.** Two roots exist; the code takes the one with the **smaller
`|θ_i|`**, which is the root nearest the home (fully lowered) position and folds up
rather than inverting through the top as the target crosses the axis. This matches
the paper's per-limb heuristic and is **hard-coded**, not configurable.

The result (`stage1_thetas_[leg]`, radians) is fed to stage 2.

> **Equivalence.** This is the same equation the previous `acos`-based routine
> solved — `DeltaArm.StageOneMatchesTheLegacyClosedForm` pins the two against each
> other and they agree to <0.02° over a pose grid. The rewrite changed how the
> maths is written down and commented, not what the arm does.

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
|a − b| ≤ |P − O| ≤ a + b
```

> This is the **complete** closure condition for this four-bar, and it is the
> single predicate shared by both directions of the solve, so the IK and the FK
> can never disagree about which arm angles are reachable.
>
> For gen0, `|a−b| = 25` and `a+b = 95`; at `θ = 0` the socket is ~92 mm from the
> servo shaft, so the arm cannot lie quite horizontal. The band does start just
> above 0°.

### 3.1a The resolved band (not a hard-coded constant)

The usable range is **not** a fixed constant. It is derived once per
`set_geometry()` by `Arm::compute_linkage_bands()` as the intersection of:

1. **closure** — `four_bar_motor_angle()` is real (§3.1);
2. **monotonicity** — the motor angle must be non-decreasing in θ. Past a
   transmission-angle *toggle* (where the two horn pins coincide) the map doubles
   back on itself and no single-valued inverse exists, so that region is unusable.
   The scan therefore takes the **longest strictly monotone sub-interval** of the
   closure run;
3. **servo travel** — the commanded motor angle (after `home_offset`) must lie in
   `geometry.angle_min .. geometry.angle_max`;
4. **arm-angle window** — `geometry.arm_angle_min .. geometry.arm_angle_max`
   (default `0 .. 180°`).

The shipped assemblies bound their bands for *different* reasons:

```
default : arm [ 31.81°, 124.77°]   servo [ 20.22°, 133.07°]   both edges = the linkage
gen0    : arm [  0.01°,  78.23°]   servo [ 15.87°, 144.89°]   top edge = the 145° servo stop
                                                               bottom edge = the linkage
```

On gen0 that puts the top of travel at arm 78.2° / servo 145°, and
`arm ≈ 6.7° per 10° of servo` over most of the band. The arm does *not* stop at
90°; on gen0 the servo does.

Opening gen0's arm-angle window to `−30°` extends the band to `−6.6°`, where the
**0° servo stop** takes over as the lower bound — the transmission itself stays
monotone through negative arm angles.

The band is logged at startup per leg and is the single authority for both
directions of the solve — see §6.

### 3.2 Exact solve — circle–circle intersection

The horn pin `E` is the intersection of the circle of radius `a` (horn) about the
servo shaft `O` with the circle of radius `b` (rod) about the arm socket `P`:

```
d      = |P − O|
along  = (d² + a² − b²) / (2d)                 projection of the a-leg onto d
h      = sqrt(a² − along²)                     altitude of the triangle (a, b, d)
u      = (P − O) / d                           unit vector O → P

E = O + along·u  −  h·(−u_y, u_x)
```

The servo angle is then read straight off the pin, measured from the horizon and
growing **downward**, and folded into `[0, 2π)`:

```
motor_rad = atan2(servo_z − E_y, E_x − servo_radius)
```

> **Why this replaced the closed form.** The previous implementation used a
> Grashof `atan2`/`acos` composition over `(a, b, c, d)`. It described no rigid
> linkage: sweeping the arm in small steps and differencing its output gave crank
> increments drifting from about **+3° to −13°** instead of staying at 0, so the
> horn was effectively stretching as the arm moved. It also admitted arm angles
> where the `acos` argument had no real value, which is why the closure test above
> used to be a separate, redundant guard.
>
> `DeltaArm.FourBarSolveIsGeometricallyExact` re-derives the pin independently and
> checks that both link lengths hold to <1e-3 mm, that the motor angle is monotone
> across the band, and that the library's own inverse returns the arm angle it
> started from.

**Assembly mode.** The two pins coincide only at a toggle (`h = 0`), which the band
scan already rejects. The machine is built with the pin on the **clockwise** side
of the directed line `O → P`; that is the branch whose servo angle rises
monotonically as the arm sweeps down, while the other is the fold-through-the-top
branch. The rule is therefore continuous over every valid band.

### 3.2.1 `bypass_reachability` clamps to the closure span

`bypass_reachability` resolves an out-of-envelope target to *the nearest arm
angle at which the linkage merely closes*, deliberately ignoring the band, so
the matching motor angle can land outside the servo's travel — gen0 produced
`230.69°`, or `356.80°` for a shallow goal, neither of which the mechanism can
hold.

`ik_stage2()` therefore clamps the **commanded** angle (post-`home_offset`, the
same quantity `compute_linkage_bands()` tests and the driver limits) into

```
[motor_angle_min, motor_angle_max]  ∩  closure span
```

Both bounds are required. The travel alone is not enough: it permits angles
below the linkage's low closure limit, and clamping to it alone yields e.g.
`6.17°`, which the 4-bar still cannot resolve. For gen0 the intersection is the
closure span `[15.87°, 144.89°]`, and bypassing past either end of the envelope
now pins to a real band edge:

| request (gen0, on-axis) | commanded servo |
|---|---|
| `z = −161` … `−345` (in envelope) | `16.41°` … `138.31°`, monotonic, unclamped |
| `z = −16` (shallow) | span edge (`15.87°` or `144.89°`) |
| `z = −350` … `−400` (deep) | `144.89°` |

Which span edge a *shallow* goal picks is not deterministic by design: the folded
raw angle jumps from ≈`15°` to ≈`356°` between neighbouring depths, so either
saturation is a legitimate "nearest closing angle" outcome. The **deep** end is
deterministic. The clamp is a no-op for in-band poses, which satisfy both bounds
by construction, so the reachable envelope is bit-for-bit unchanged.

## 3.3 Worked values

Default assembly (`arm/config/arm_params.yaml`):

| Arm θ (deg) | Servo (deg) | Note |
|---|---|---|
| 31.81 (band_low) | 20.22 | linkage closes here |
| 37.7 | 35.20 | live goal `(0,0,−0.25)` |
| 45 | 46.35 | |
| 48.7 | 51.40 | initial `(0,0,−0.28)` |
| 56.8 | 61.78 | live goal `(0,0,−0.30)` |
| 60 | 65.71 | |
| 75 | 83.37 | |
| 90 (down) | 100.20 | |
| 124.77 (band_high) | 133.07 | linkage closes here |

gen0 assembly (`arm/config/arm_gen0_params.yaml`, which uses `lower_arm_len = 250`),
on-axis goals:

| Goal `(0,0,z)` | Arm θ (deg) | Servo (deg) |
|---|---|---|
| −0.166 | 0.30 | 16.41 |
| −0.170 | 2.17 | 19.77 |
| −0.180 | 6.58 | 27.20 |
| −0.200 | 14.64 | 39.80 |
| −0.250 | 32.87 | 66.93 |
| −0.280 | 43.84 | 83.46 |
| −0.300 | 51.69 | 95.77 |
| −0.320 | 60.49 | 110.36 |
| −0.340 | 71.24 | 130.12 |
| −0.346 | 74.53 | 138.31 |
| −0.351 | *refused* | needs arm 79.10° → past the 145° stop |

So the on-axis envelope is **z ∈ [−348, −163] mm**, and **both** of its ends are
set by the 4-bar band, not by the stage-1 circle:

* shallow end — `z < −163 mm` needs a *negative* arm angle, refused by the band's
  lower edge (`arm < 0.01°`). The circle miss only takes over below −111 mm.
* deep end — `z > −348 mm` needs `arm > 78.23°`, i.e. servo `> 145°`. The stage-1
  circle only gives out further down, at about −363 mm.

Across the whole envelope the servo angle is strictly monotonic
(`16.41° → 138.31°`) and always inside the closure span `[15.87°, 144.89°]`.
These numbers were confirmed end-to-end through the running ROS stack
(`gen0`, 2-D virtual), where the drawn pose matched the request to `< 0.001 mm`.

`motor_from_arm` output (before `home_offset`): `deg` = `(motor_rad + home_offset)·180/π`.

> These servo angles come from the geometry as configured and **not** from a
> hardware calibration. `home_offset` is still 0, so the absolute servo figures
> carry whatever error the measured horn/rod lengths have; see the README's
> calibration note.

---

## 4. Inverse 4-bar (`arm_from_motor`, used by FK)

`motor_from_arm` is monotone increasing across the **resolved band** (§3.1a), so
the inverse is a simple 60-iteration **bisection** on θ that re-evaluates the
geometric solve — never an algebraic inversion. Bisection also keeps the two
directions of the solve tied together by construction: both call the same
`four_bar_motor_angle()`, so they cannot drift apart.

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