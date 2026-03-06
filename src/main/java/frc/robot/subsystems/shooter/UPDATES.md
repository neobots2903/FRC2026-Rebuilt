# Shooter Kinematics — Solving for Both Angle AND Velocity

## The Problem We Had

Our original code fixed the flywheel at a constant RPM and only solved for the hood angle.
This caused `NaN` (no valid solution) at many distances because **a single velocity can't reach every target**.

### Why?

The projectile motion quadratic for launch angle $\theta$ is:

$$k \tan^2(\theta) - d \tan(\theta) + (\Delta z + k) = 0$$

where $k = \frac{g d^2}{2 v^2}$.

The discriminant is:

$$\Delta = d^2 - 4k(\Delta z + k)$$

If the exit velocity $v$ is too low, $k$ becomes large and $\Delta < 0$ — **no real solution exists**.
Even when $\Delta \geq 0$, both angles may fall outside the hood's mechanical limits or fail the
top-opening descent check.

### The Fix

Instead of fixing velocity and solving for angle, we:

1. **Compute the minimum exit velocity** needed to reach the target (from the $\Delta \geq 0$ condition).
2. **Sweep velocities** from that minimum up to the motor's maximum.
3. At each velocity, solve the quadratic for angle.
4. **Return the first valid (velocity, angle) pair** — lowest energy, most accurate.

---

## The Math

### Minimum Exit Velocity

For the discriminant to be non-negative:

$$v^2 \geq g \left(\Delta z + \sqrt{\Delta z^2 + d^2}\right)$$

Therefore:

$$v_{\min} = \sqrt{g \left(\Delta z + \sqrt{\Delta z^2 + d^2}\right)}$$

### Exit Velocity ↔ Motor RPM Conversion

The ball exits at half the wheel surface speed (because it rolls against a stationary hood):

$$v_{\text{exit}} = \frac{1}{2} \cdot \frac{\text{RPM}_{\text{wheel}}}{60} \cdot 2\pi r$$

$$\text{RPM}_{\text{wheel}} = \frac{\text{RPM}_{\text{motor}}}{\text{gear reduction}}$$

To convert back:

$$\text{RPM}_{\text{motor}} = \frac{2 \cdot v_{\text{exit}}}{2\pi r} \cdot 60 \cdot \text{gear reduction}$$

### Angle Selection

When both quadratic roots are valid, we prefer the angle **closest to 45°** (the optimal projectile angle
that maximizes range for a given velocity). This tends to give the most forgiving trajectory.

---

## Result Class

Instead of returning just a `double` angle, we return a `ShooterSolution`:

| Field | Type | Meaning |
|-------|------|---------|
| `hoodAngleDeg` | `double` | The hood angle to set |
| `motorRPM` | `double` | The flywheel RPM to spin at |
| `isValid` | `boolean` | Whether the target is reachable |

---

## Algorithm Pseudocode

```
function getShooterSolution(robotPose, targetPos):
    distance = getDistanceToTarget(robotPose, targetPos)
    deltaZ = targetHeight - shooterHeight
    vMax = getMaxExitVelocity()
    vMin = max(getMinExitVelocity(distance, deltaZ), MIN_VELOCITY)

    if vMin > vMax:
        return INVALID  // target is unreachable

    for v from vMin to vMax step 0.5:
        angle = solveQuadraticForAngle(v, distance, deltaZ)
        if angle is valid:
            rpm = convertToMotorRPM(v)
            return Solution(angle, rpm, true)

    return INVALID
```

---

## Debugging History

| Session | Problem | Root Cause | Fix |
|---------|---------|------------|-----|
| 1 | `exitVelocity = 0.0` | `return 0.0` instead of computed value | Return `surfaceSpeed / 2.0` |
| 2 | `exitVelocity = 603.9` (absurd) | Missing RPM → RPS conversion (÷60) | `realRPM / 60.0` |
| 3 | `discriminant = -1553` | 5.03 m/s too slow for 10 m distance | Changed gear reduction |
| 4 | `discriminant = -9.6` | Still too slow for 2.76 m + 1.45 m height | Adjusted gear ratio |
| 5 | `angle2 = 34.4°` in range but trajectory invalid | Ball still rising at target (too flat) | — |
| 6 | `angle1 = 82.2°` valid physics but outside limits | `MAX_HOOD_ANGLE_DEG = 63°` too narrow | — |
| **Final** | **Fixed RPM can't solve all distances** | **Fundamental design flaw** | **Solve for both RPM and angle** |

## Look-Up Table (LUT)

`InterpolatingDoubleTreeMap` maps `double → double`, so it can only store **one** output per key.
To store both angle and RPM, we use **two parallel maps**, both keyed by distance:

| Map | Key | Value |
|-----|-----|-------|
| `HOOD_ANGLE_MAP` | distance (m) | hood angle (°) |
| `FLYWHEEL_RPM_MAP` | distance (m) | motor RPM |

Both maps use WPILib's built-in linear interpolation, so you only need to populate a handful of
tested data points. The map will smoothly interpolate between them.

### How to populate (at the field)

1. Place the robot at a known distance from the target.
2. Manually tune angle and RPM until you consistently score.
3. Record the values:
   ```java
   HOOD_ANGLE_MAP.put(2.0, 55.0);    FLYWHEEL_RPM_MAP.put(2.0, 2500.0);
   HOOD_ANGLE_MAP.put(3.0, 45.0);    FLYWHEEL_RPM_MAP.put(3.0, 3000.0);
   ```
4. Repeat at 4–6 different distances across your shooting range.
5. Set `ENABLE_EMPIRICAL_LUT = true` to switch from physics to tested values.

### Why two maps instead of a custom class?

- `InterpolatingDoubleTreeMap` handles interpolation automatically — no custom code needed.
- Both maps interpolate independently, which is fine since angle and RPM scale differently with distance.
- It's the simplest approach that WPILib supports out of the box.
