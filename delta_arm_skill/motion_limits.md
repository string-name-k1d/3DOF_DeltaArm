# Motion limits and recorded behaviour

Source: `arm_log_0.db3` in `/home/k1d/Code/fyp/arm_log` (997 `MotorFeedback`
samples, 99.6 s, all servos online, no errors).

## Measured motor range

| Motor | Observed min | Observed max | Plateau evidence |
|---|---:|---:|---|
| 0 | 28.7° | 133.0° | 133.0° held ~1 s; 132.5° held ~1.8 s. The 28.7° sample is isolated, not a stop. |
| 1 | 20.7° | 131.9° | 20.7° held ~10.1 s; 131.9° held ~1.2 s. |
| 2 | 21.0° | 131.5° | 21.2° held ~10.1 s; 131.1° held ~13.7 s. |

Combined usable window: roughly **20.7° .. 133.0°**, not `0..145`.

## Derived bands from the current model

Running the actual `arm_kinematics` code with the default geometry:

| Config | Arm-angle band | Motor span |
|---|---:|---:|
| `[0..145]` | 31.81° .. 124.77° | 20.22° .. 133.07° |
| `[20.8,20.7,21] .. [133,132,131.5]` | leg0 31.93..124.15°; leg1 31.91..121.91°; leg2 31.95..121.13° | 21.22..132.99°; 21.08..131.96°; 21.35..131.46° |
| `[0..360]` | 31.81° .. 124.77° | 20.22° .. 133.07° |

The wide and current windows give the same band: the four-bar itself peaks at
**133.07° motor angle**, so `angle_max: 145` is physically unreachable for this
geometry. Lowering the configured max to ~133 does not lose workspace.

## Log mapped through FK

Observed arm angles: motor0 34.38..127.13°, motor1/2 31.88..127.13°.

FK pose envelope:

```text
x -0.128..0.104 m, y -0.129..0.177 m, z -0.326..-0.245 m
max radial 0.177 m
```

Plateau poses:

| Motors | Arm angles | Position |
|---|---|---|
| `(33.8,41.0,22.2)` | `(36.9,41.4,32.1)°` | `(-0.016,0.000,-0.247)` m |
| `(56.5,73.2,131.1)` | `(52.6,66.3,120.6)°` | `(0.090,-0.077,-0.308)` m |
| `(46.7,20.7,21.2)` | `(45.3,31.9,31.9)°` | `(0.000,0.028,-0.245)` m |

## Branch/fold issue

The 4-bar motor angle is not globally monotone:

- rises to 133.07° at arm ≈124.82°;
- folds, then changes branch past that point;
- motor 131.1° has solutions near arm 120.6° and ~126.8°.

`arm_from_motor()` in raw mode scans the whole `0..180°` arm window, so it can
pick the folded branch near the top of the range. Samples above 128°:

```text
motor0 122, motor1 27, motor2 195
```

Worst arm-angle difference between full-window inverse and the monotone command
branch is about **10°**. Fix by inverting only the monotone command branch
(~31.8..124.8°) or by tracking the branch from the commanded stage-1 angle.

## Candidate limits

For a temporary empirical raw clamp:

```yaml
angle_min: [20.8, 20.7, 21.0]
angle_max: [133.0, 132.0, 131.5]
```

Geometric intrinsic limits are closer to `[20.2]*3 .. [133.1]*3`. Do not set
motor0's lower limit from its isolated 28.7° reading.
