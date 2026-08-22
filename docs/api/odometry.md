---
layout: default
title: Odometry
parent: API Reference
nav_order: 2
---

# Odometry
{: .no_toc }

How RW 2.0 knows where the robot is, what the coordinate numbers mean, and how to correct them mid-routine.
{: .fs-6 .fw-300 }

<details open markdown="block">
  <summary>On this page</summary>
  {: .text-delta }
- TOC
{:toc}
</details>

---

## The coordinate frame

Get this straight before anything else, because the frame is **not** the standard math convention.

```
                    +Y
                     |
                heading 0°
                     |
      heading 270° --+-- heading 90°     +X
                     |
                heading 180°
                     |
                    -Y
```

- **Heading 0° faces +Y.** Not +X.
- **Heading increases clockwise.** 90° faces +X, 180° faces −Y, 270° faces −X.
- **Units are inches** for position, degrees for heading.
- **The origin is wherever the robot was when tracking started.** It is not the center of the field.

Compass-style navigation, and the position math bears that out. From `trackNoOdomWheel()`:

```cpp
x_pos += polar_radius_in * sin(polar_angle_rad);
y_pos += polar_radius_in * cos(polar_angle_rad);
```

`sin` on X and `cos` on Y is the swap that produces a clockwise-from-+Y frame.

{: .tip }
> **Place the origin deliberately.** Odometry starts at (0, 0) with heading 0 wherever you set the robot down. If you want your coordinates to match field coordinates, either start the robot at a known field position and offset your targets by hand, or square up against a wall and call a [position reset](#position-resets) at the top of your routine.

## The four tracking modes

You never call these directly. `trackPosition()` in [`src/main.cpp`](https://github.com/Master101215/RW-2.0/blob/main/src/main.cpp) reads the two `using_*_tracker` booleans from `robot-config.cpp` and starts the right one as a background thread. All four update every 10 ms.

| Mode | Started when | Reads | Notes |
|:-----|:-------------|:------|:------|
| `trackXYOdomWheel` | both trackers enabled | Both rotation sensors + IMU | Most accurate. Detects sideways motion. |
| `trackXOdomWheel` | horizontal only | Horizontal tracker + drive encoders + IMU | Forward travel from the drivetrain |
| `trackYOdomWheel` | vertical only | Vertical tracker + IMU | No lateral detection |
| `trackNoOdomWheel` | neither | Drive encoders + IMU | Fallback. No lateral detection. |

All four integrate along an arc instead of assuming straight-line motion between samples, so tight curves track correctly instead of accumulating error.

### What "no lateral detection" costs you

Only `trackXYOdomWheel` sees the robot move sideways. Without a horizontal tracker, a shove from another robot or a wheel skidding across the field registers as *nothing*. The encoders don't turn, so odometry believes the robot never moved.

That's the case for [position resets](#position-resets), and for testing your routine against contact rather than on an empty field.

## Position globals

```cpp
extern double x_pos, y_pos;      // inches
extern double correct_angle;     // degrees, target heading for correction
extern bool is_turning;          // true while a motion function is running
```

Read them any time:

```cpp
if (hypot(24 - x_pos, 36 - y_pos) < 6) {
  // within six inches of the target
}
```

Writing `x_pos` and `y_pos` directly is how the reset functions work, and it's fair game if you know the robot's true position:

```cpp
x_pos = 0;
y_pos = 0;    // declare "here is the origin"
```

`correct_angle` is the heading the correction PID holds. `auton()` initializes it with `correct_angle = normalizeTarget(0)`, and motion functions update it as they finish.

## Heading functions

```cpp
double getInertialHeading();
```
Current heading in degrees from the inertial sensor.

```cpp
double normalizeTarget(double angle);
```
Wraps a target angle to within ±180° of the current heading, so a turn takes the short way around. `turnToAngle` calls it internally, so you need it only when you set `correct_angle` yourself.

```cpp
void correctHeading();
```
Runs one iteration of the heading-correction PID against `correct_angle`. The template uses it to hold heading while the robot sits still, and the `heading_correction` flag in `robot-config.cpp` gates it.

## Position resets

Odometry drifts. Wheels slip, the IMU accumulates a fraction of a degree per second, and a hard collision can throw the estimate off by inches. Over a 15-second autonomous that adds up.

If you have distance sensors, you can snap position back to truth against a field wall:

```cpp
void resetPositionFront();
void resetPositionBack();
void resetPositionLeft();
void resetPositionRight();
```

Each one reads its sensor, works out from the current heading which wall it faces, and overwrites **either `x_pos` or `y_pos`**, never both. A front sensor facing the +Y wall fixes `y_pos` and leaves `x_pos` alone.

To fix both axes, square up and use two sensors on perpendicular walls:

```cpp
turnToAngle(0, 1000);      // square to the wall
driveTo(-6, 1000);         // back up close to it
resetPositionBack();       // fixes y_pos
resetPositionRight();      // fixes x_pos, if that sensor faces a side wall
```

### Rules for resets

{: .warning }
> **Only call these when the robot is square to a wall.** The math corrects for small heading error with a cosine term, but a robot at 30° to the wall is measuring a diagonal, and the correction degrades fast. Turn to a cardinal heading first.

{: .warning }
> **Never call `resetPositionWithSensor()` directly.** It's the shared implementation behind the four wrappers, and the version declared in the header is not the version that exists. Use the four direction functions.

Other things to know:

- **Bad readings go nowhere.** A reading below 0 or above 200 inches returns without touching position, so a sensor that sees nothing won't corrupt your estimate.
- **Sensor offsets must be right.** The `*_offsetX` / `*_offsetY` values in `robot-config.cpp` are how the code converts "sensor is 14″ from the wall" into "robot center is at Y = 55.75". An unset offset puts that much error into the reset. See [Configuration]({{ site.baseurl }}/configuration#distance-sensor-offsets).
- **Field half-size is fixed at 70.25 inches**, set in `motor-control.cpp`. That's the distance from field center to wall. Change it there if you're running on a non-standard field.
- **Reset before you need accuracy.** A reset just ahead of a scoring move buys far more than one at the end of the routine.

## Debugging odometry

The template runs a `logPosition()` task that prints to the VS Code terminal every 100 ms:

```
BEGIN RUN
X: 0.00 Y: 12.03 Heading: 0.15
X: 0.00 Y: 18.44 Heading: 0.22
```

This needs the controller connected by USB cable. The same values show up live on the controller screen (`X:` / `Y:` line) and on the brain (`ODOM x: y: h:`).

**A sanity check before you trust any of it:** put the robot on the floor, download, and push it forward 24 inches by hand. `Y` should read close to 24. If it reads 22, `wheel_distance_in` is about 8% low. Then spin it 360° by hand. Heading should come back to roughly 0, and X/Y should barely move. If position wanders during a pure rotation, the tracker `dist_from_center` offsets are wrong.