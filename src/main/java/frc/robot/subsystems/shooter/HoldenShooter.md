# Shooter Subsystem

## Original Notes

Holden's ideas are still in the planning process:

> Shooter (maybe turret, maybe stationary) starts by having balls go into a flywheel, which sends them up. Then they are in the top bit, which then has another flywheel that shoots. Also there's a flywheel inside the bot that helps the balls move along.

**Our planning process:** Since they are still planning, just make a basic skeleton and set up the motors; also study documentation to know what to do.

---

## Notes (since I will be gone Wed–Fri)

TO VIEW THIS PROPERLY, RIGHT CLICK THE FILE AND HIT OPEN PREVIEW!!!

### Action Plan

1. Implement `ShooterKinematics.java`
2. Create the IO interface and hardware files
3. Create `Shooter.java` subsystem
4. Update `RobotContainer.java` to instantiate shooter, this needs a pose supplier (can't be single value, needs supplier so it always has updated values `drive::getPose`)
5. Add controls and test

### Hardware

| Mechanism  | Motor              | Controller  |
| ---------- | ------------------ | ----------- |
| Flywheel   | Unknown — possibly 1× Neo 1.1 or 1× Kraken X60 | Spark Max / TalonFX |
| Hood       | Probably 1× Neo 550 | Spark Max   |
| Rotation   | Probably 1× Neo 1.1 | Spark Max   |

### Control

- **Flywheel:** Velocity PID Control (keep the motor at its peak-power RPM)
- **Hood & Rotation:** Position PID Control (just like the intake pivot — hold position)

### Math

#### Flywheel

We might be able to get away with keeping RPM constant (or within a small range) and adjusting only hood and rotation. For a Neo 1.1, peak power is at around **3000 RPM**, so we'd want to target right around that. Once we can do real testing, we can move to RPM zones, or add RPM to the interpolation table.

#### Hood (Darius can help with math)

**Primary control — lookup table with linear interpolation:**

1. Place the robot at different distances from the hub and shoot.
2. Record the distance and the hood angle that works best at each distance.
3. Build a table of `(distance, angle)` pairs; the program will linearly interpolate to find the optimal angle even between measured points.

**Secondary control — physics (since we likely won't be able to test much):**

Start with the standard 2D projectile trajectory equation:

$$y = x \tan(\theta) - \frac{g x^2}{2 v^2 \cos^2(\theta)}$$

Where:

| Variable | Meaning | Units |
| -------- | ------- | ----- |
| $y$ | Vertical displacement — height difference between the shooter and the target (hub) | meters |
| $x$ | Horizontal distance from the shooter to the target | meters |
| $\theta$ | Launch angle of the ball (the hood angle we're solving for) | radians |
| $v$ | Initial velocity of the ball as it leaves the flywheel | m/s |
| $g$ | Acceleration due to gravity ($\approx 9.81 \, \text{m/s}^2$) | m/s² |


Using the identity $\frac{1}{\cos^2(\theta)} = 1 + \tan^2(\theta)$ and substituting $t = \tan(\theta)$:

$$y = x \, t - \frac{g x^2}{2 v^2}(1 + t^2)$$

Rearranging into standard quadratic form $at^2 + bt + c = 0$:

$$\frac{g x^2}{2 v^2} \, t^2 - x \, t + \left(y + \frac{g x^2}{2 v^2}\right) = 0$$

Solving with the quadratic formula and converting back:

$$\theta = \arctan\left(\frac{x \pm \sqrt{x^2 - 4 \cdot \frac{g x^2}{2 v^2} \cdot \left(y + \frac{g x^2}{2 v^2}\right)}}{2 \cdot \frac{g x^2}{2 v^2}}\right)$$

The two solutions represent:

- **`−` root:** Low-arc trajectory (preferred for speed)
- **`+` root:** High-arc trajectory

#### Bounce-Out Detection (Top-Opening Validation)

Not every valid launch angle will actually score. If the ball arrives too flat (nearly horizontal), it will hit the rim and bounce out instead of dropping into the hub's top opening. The [`isTrajectoryValidForTopOpening`](src/main/java/frc/robot/subsystems/shooter/ShooterKinematics.java) method in [`ShooterKinematics`](src/main/java/frc/robot/subsystems/shooter/ShooterKinematics.java) checks this.

**The idea:** simulate the ball's vertical velocity at the moment it reaches the target and verify it is falling steeply enough.

**Steps:**

1. **Decompose the exit velocity** into horizontal and vertical components:

$$v_x = v \cos(\theta), \quad v_{iy} = v \sin(\theta)$$

2. **Find Time of Flight** — the time for the ball to travel horizontal distance $x$:

$$t = \frac{x}{v_x}$$

3. **Find final vertical velocity** at the target using kinematics:

$$v_{fy} = v_{iy} - g \, t$$

4. **Check the ball is falling** — if $v_{fy} > 0$, the ball is still rising at the target. That means it hasn't reached the apex yet and will overshoot. **Reject this shot.**

5. **Calculate the entry (descent) angle:**

$$\alpha = \left| \arctan\left(\frac{v_{fy}}{v_x}\right) \right|$$

6. **Compare against the minimum descent angle.** The hub opening has a diameter of **`TARGET_DIAMETER_IN`** ($24''$). To reliably drop in, we require:

$$\alpha \geq \text{MIN\_DESCENT\_ANGLE\_DEG} \; (20°)$$

   If the entry angle is too shallow, the ball skims across the opening and bounces out. Only shots where $\alpha$ meets the threshold are accepted.

**In [`ShooterKinematics`](src/main/java/frc/robot/subsystems/shooter/ShooterKinematics.java):**

- [`getPhysicsHoodAngleDeg`](src/main/java/frc/robot/subsystems/shooter/ShooterKinematics.java) solves the quadratic for both roots, converts them to degrees, checks they are within `MIN_HOOD_ANGLE_DEG`–`MAX_HOOD_ANGLE_DEG`, **and** calls `isTrajectoryValidForTopOpening` on each candidate before accepting it. It prefers the flatter (smaller) valid angle.
- [`getOptimalHoodAngleDeg`](src/main/java/frc/robot/subsystems/shooter/ShooterKinematics.java) acts as the top-level entry point — it will use the empirical look-up table when available, and falls back to the physics solver (which includes bounce-out detection) otherwise.

---

### Example Usage (not real code)

```java
public void periodic() {
    // 1. Get our current position from the Swerve Drive
    Pose2d currentPose = swerveDrive.getPose();

    // 2. Ask the math class for the required angles
    double targetTurretAngle = ShooterKinematics.calculateTurretAngleDeg(currentPose);
    double targetHoodAngle = ShooterKinematics.getOptimalHoodAngleDeg(currentPose);

    // 3. Safety Check: Are we in a blind spot or out of range?
    if (Double.isNaN(targetTurretAngle) || Double.isNaN(targetHoodAngle)) {
        // DO NOT SHOOT. Target is impossible to hit.
        feeder.stop();
        hood.setAngle(0); // Park the hood
    } else {
        // 4. Aim the mechanisms!
        turret.setTargetAngleDeg(targetTurretAngle);
        hood.setTargetAngleDeg(targetHoodAngle);
        flywheel.setTargetRPM(ShooterKinematics.IDEAL_MOTOR_RPM);
    }
}
```

