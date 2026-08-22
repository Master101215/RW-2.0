---
layout: default
title: Motion
parent: API Reference
nav_order: 1
---

# Motion
{: .no_toc }

[`include/motor-control.h`](https://github.com/Master101215/RW-2.0/blob/main/include/motor-control.h) declares these and [`src/motor-control.cpp`](https://github.com/Master101215/RW-2.0/blob/main/src/motor-control.cpp) implements them. Each one blocks until the movement finishes or its time limit expires.
{: .fs-6 .fw-300 }

<details open markdown="block">
  <summary>On this page</summary>
  {: .text-delta }
- TOC
{:toc}
</details>

---

## Shared parameters

Most motion functions take the same four trailing parameters. They mean the same thing everywhere.

`time_limit_msec`
: A **hard timeout**, not a target. The function returns when the PID settles *or* when this many milliseconds have passed, whichever lands first. Set it generously. A move that times out stops wherever it happens to be, and the rest of your routine then runs from the wrong place. If a drive usually takes 1.2 s, give it 2000.

`exit`
: `true` (default) stops the robot with `hold` braking at the end. `false` hands off to the next command without stopping. See [Motion chaining](#motion-chaining).

`max_output`
: Voltage ceiling, 0 to 100. When the outputs would exceed it, the code scales both sides down together and keeps their ratio, so the robot doesn't veer at saturation.

`dir`
: `1` to approach driving forward, `-1` to approach in reverse. A reverse approach often beats turning around first.

`overturn`
: When the turn correction and the forward drive together exceed `max_output`, `overturn = true` sacrifices forward speed so the turn keeps full authority. Use it when the robot swings wide on sharp approaches. Default `false`.

---

## driveTo

```cpp
void driveTo(double distance_in, double time_limit_msec,
             bool exit = true, double max_output = 100);
```

Drives straight for a signed distance, holding the current heading with the heading-correction PID.

| Parameter | Meaning |
|:----------|:--------|
| `distance_in` | Inches. Positive drives forward, negative drives in reverse. |

```cpp
driveTo(24, 2000);        // forward two feet
driveTo(-12, 1500);       // back up one foot
driveTo(36, 3000, true, 60);   // forward three feet, capped at 60% output
```

Accuracy here comes down to `wheel_distance_in`. See [Configuration]({{ site.baseurl }}/configuration#required-measurements).

## turnToAngle

```cpp
void turnToAngle(double turn_angle, double time_limit_msec,
                 bool exit = true, double max_output = 100);
```

Turns in place to an **absolute field heading** rather than a relative amount. `turnToAngle(90, 1000)` ends facing 90° whatever the robot started at.

The code normalizes the target to within ±180° of the current heading, so the robot takes the short way around.

```cpp
turnToAngle(90, 1000);     // face 90°
turnToAngle(-45, 1000);    // face -45° (equivalently 315°)
turnToAngle(0, 1000);      // return to the starting heading
```

{: .note }
> Heading 0° is whatever direction the robot faced when `auton()` started — the IMU is zeroed there. See [Odometry]({{ site.baseurl }}/api/odometry#the-coordinate-frame).

## swing

```cpp
void swing(double swing_angle, double drive_direction, double time_limit_msec,
           bool exit = true, double max_output = 100);
```

A swing turn. The robot rotates toward a heading while driving, pivoting around one side instead of spinning on its center. It beats turn-then-drive when you need to change heading and make progress at once.

| Parameter | Meaning |
|:----------|:--------|
| `swing_angle` | Target heading in degrees, absolute |
| `drive_direction` | `1` to swing while moving forward, `-1` while moving backward |

```cpp
swing(90, 1, 1500);     // swing forward-right to face 90°
swing(-90, -1, 1500);   // swing backward to face -90°
```

## curveCircle

```cpp
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec,
                 bool exit = true, double max_output = 100);
```

Drives a constant-radius arc until the robot reaches the target heading.

| Parameter | Meaning |
|:----------|:--------|
| `result_angle_deg` | Heading the robot should end at, in degrees |
| `center_radius` | Arc radius in inches. **Positive curves right, negative curves left.** |

```cpp
curveCircle(90, 24, 2500);    // arc right on a 24" radius, ending at 90°
curveCircle(-90, -24, 2500);  // arc left on a 24" radius, ending at -90°
```

{: .tip }
> `curveCircle` uses only heading and distance PID — **no odometry at all**. If your robot has no tracking wheels and you don't trust `x_pos`/`y_pos` yet, this is how you get curved paths.

## turnToPoint

```cpp
void turnToPoint(double x, double y, int dir, double time_limit_msec);
```

Turns in place until the robot faces a field coordinate. It needs odometry, since it computes the bearing from `x_pos`/`y_pos`.

| Parameter | Meaning |
|:----------|:--------|
| `x`, `y` | Field coordinates of the point to face, in inches |
| `dir` | `1` to point the front at it, `-1` to point the back at it |

```cpp
turnToPoint(24, 48, 1, 1500);    // face the point
turnToPoint(24, 48, -1, 1500);   // back toward the point, ready to reverse into it
```

{: .note }
> Unlike the others, this takes no `exit` or `max_output` parameter.

## moveToPoint

```cpp
void moveToPoint(double x, double y, int dir, double time_limit_msec,
                 bool exit = true, double max_output = 100, bool overturn = false);
```

Drives to a field coordinate, re-aiming as it goes. It ends when the robot crosses the line perpendicular to its approach through the target, so the robot stops *at* the point instead of circling it.

Heading correction runs only while the robot sits more than 8 inches out. Inside that, it drives straight in rather than fighting for a perfect bearing over a few inches.

```cpp
moveToPoint(24, 24, 1, 3000);                    // drive to (24, 24)
moveToPoint(0, 0, -1, 3000);                     // reverse back to the origin
moveToPoint(48, 12, 1, 3000, true, 80, true);    // capped at 80%, overturn on
```

The final heading is whatever it ends up being. If you need a specific one, use `boomerang`.

## boomerang

```cpp
void boomerang(double x, double y, int dir, double a, double dlead,
               double time_limit_msec, bool exit = true,
               double max_output = 100, bool overturn = false);
```

Drives to a **pose**, meaning a coordinate *plus* a final heading, along a curved path. The controller aims at a moving carrot point ahead of the target. That point pulls in as the robot approaches, so the robot arrives lined up instead of turning at the end.

| Parameter | Meaning |
|:----------|:--------|
| `x`, `y` | Target coordinates, in inches |
| `dir` | `1` forward, `-1` reverse |
| `a` | Final heading at the target, in degrees |
| `dlead` | How far ahead the carrot leads. Higher = curvier approach. |

{: .warning }
> **Watch the parameter order: `dir` comes before `a`.** It is `(x, y, dir, a, dlead, ...)`, not `(x, y, a, dlead, ...)`. Getting these backwards compiles cleanly and drives somewhere unexpected.

{: .warning }
> **Do not set `dlead` above 0.6.** Values in the 0.3–0.6 range are typical; higher ones make the path curve so aggressively the controller loses the target.

```cpp
// Drive to (24, 36) arriving at 90°, moderately curved
boomerang(24, 36, 1, 90, 0.5, 3000);

// Tighter, straighter approach
boomerang(24, 36, 1, 90, 0.3, 3000);
```

If the robot slips or drifts on the curve, lower `chase_power` in `robot-config.cpp`. It limits how hard boomerang can cut the corner. See [Configuration]({{ site.baseurl }}/configuration#advanced-tuning).

---

## Motion chaining

Pass `exit = false` and the robot rolls straight into the next command instead of stopping at the end of the movement. Routines built this way beat stop-and-go ones, because the drive never re-accelerates from zero.

```cpp
driveTo(24, 2000, false);        // roll through
turnToAngle(90, 1000, false);    // roll through
driveTo(18, 1500);               // exit defaults to true — this one stops
```

{: .warning }
> **The last command in a chain must use `exit = true`** (or just omit it). Otherwise the routine ends with the motors still commanded and the robot keeps going.

### What changes when you chain

A chained movement behaves differently from a normal one:

- **No terminal stop.** The code skips the `stopChassis(hold)` at the end, and the slew state carries into the next command.
- **The exit condition changes.** The move ends once the robot covers the target distance, without waiting for the PID to settle within tolerance.
- **No taper.** A chained `driveTo` holds full output, still clamped by `max_output`, rather than easing off on approach. Decelerating into a movement you're about to continue would waste the point of chaining.
- **`min_output` becomes a floor.** Depending on the `dir_change_start` / `dir_change_end` settings, output stays at or above `min_output` (default 5 V) so the robot doesn't stall at low error.

Chained moves don't wait to settle, so they trade precision for speed. Chain the middle of a routine, and use a normal `exit = true` move wherever the robot has to land accurately: scoring positions, alignments, contact with a wall.

{: .tip }
> `dir_change_start` and `dir_change_end` in `robot-config.cpp` tell the slew limiter whether to expect a direction change at each end of a chained move. Setting both to `false` gives the fastest, roughest chaining; `true`/`true` (the default) is the smoothest.

---

## Chassis helpers

Lower-level functions, useful for custom movements and for driver control.

```cpp
void driveChassis(double left_power, double right_power);
```
Sets both sides directly, **in volts** (−12 to 12). The motion functions all bottom out here.

```cpp
void stopChassis(vex::brakeType type = vex::brake);
```
Stops both sides. Pass `vex::coast`, `vex::brake`, or `vex::hold`. Motion functions use `hold` at the end of a non-chained move.

```cpp
void resetChassis();
```
Zeroes both drive motor encoders. Odometry calls it at startup.

```cpp
double getLeftRotationDegree();
double getRightRotationDegree();
```
Current rotation of each side in degrees. Convert to inches with `(degrees / 360.0) * wheel_distance_in`.

## A worked routine

```cpp
void auton() {
  trackPosition();
  vex::task updater(updateTask);
  vex::task logposition(logPosition);
  update();
  inertial_sensor.setRotation(0, degrees);
  correct_angle = normalizeTarget(0);

  // Rush forward and grab
  driveTo(30, 2000, false);      // chained — keep rolling
  driveTo(6, 1000);              // settle precisely on the target
  intake.spin(forward, 12, volt);
  wait(400, msec);

  // Curve out to the scoring pose
  boomerang(24, 40, 1, 45, 0.4, 3000);
  intake.stop();

  // Square up on the wall and fix odometry drift
  turnToAngle(0, 1000);
  driveTo(-8, 1200);
  resetPositionBack();

  moveToPoint(0, 12, 1, 2500);
}
```