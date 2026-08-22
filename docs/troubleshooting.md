---
layout: default
title: Troubleshooting
nav_order: 6
---

# Troubleshooting
{: .no_toc }

Known rough edges in the template, and the failure modes you're most likely to hit.
{: .fs-6 .fw-300 }

<details open markdown="block">
  <summary>On this page</summary>
  {: .text-delta }
- TOC
{:toc}
</details>

---

## Known limitations

These ship with the template. None of them break a correctly configured robot, though finding one the hard way costs an afternoon.

### Functions that are declared but not implemented

[`include/motor-control.h`](https://github.com/Master101215/RW-2.0/blob/main/include/motor-control.h) declares these, but no definition exists anywhere in `src/`. **Calling one fails at link time**, with an `undefined reference` error that points nowhere near the real problem:

```cpp
void wallDriveForwardLeft(...);
void wallDriveForwardRight(...);
void wallDriveBackwardLeft(...);
void wallDriveBackwardRight(...);
void driveStraight(double distance_in, double time_limit_msec, bool exit, double max_output);
bool waitForBlock(int targetHue, int timeoutMs);
```

For straight driving, use `driveTo()`. It holds heading with the correction PID, which is what `driveStraight` would have done.

### Port conflict in the shipped config

In [`src/robot-config.cpp`](https://github.com/Master101215/RW-2.0/blob/main/src/robot-config.cpp), the inertial sensor and the left distance sensor are both on **PORT9**:

```cpp
inertial inertial_sensor = inertial(PORT9);
...
distance left_sensor = distance(PORT9);
```

Two devices can't share a port. Move one to a free port before using `resetPositionLeft()`. With no left distance sensor at all, point `left_sensor` at any unused port and never call the function. The object gets constructed and never read.

### include/dsr.h does not compile

[`include/dsr.h`](https://github.com/Master101215/RW-2.0/blob/main/include/dsr.h) contains wall-alignment routines (`dsrBack`, `dsrFront`, `resetAngleBack`) that reference sensors named `back_left` and `back_right`. `robot-config.h` never declares those devices.

Nothing includes this file, so the build ignores it. **Do not `#include "dsr.h"` as it stands.** You'd have to declare the two sensors first. For wall-based correction, use the [position reset functions]({{ site.baseurl }}/api/odometry#position-resets) instead.

### The IMU is never calibrated

`main()` registers `auton()` and `telop()` and returns. It never calls `vexcodeInit()`, and no `pre_auton` callback exists. Both `auton()` and `telop()` call `inertial_sensor.setRotation(0, degrees)`, which zeroes the heading without running the sensor's calibration routine.

The V5 IMU works well enough in practice without an explicit calibration call. If you want one, add it to `main()` before you construct the competition object, and **wait for it to finish**. The robot has to sit completely still throughout, and moving during calibration leaves the heading skewed for the rest of the run:

```cpp
int main() {
  inertial_sensor.calibrate();
  while (inertial_sensor.isCalibrating()) { wait(50, msec); }

  competition Competition = competition();
  Competition.autonomous(auton);
  Competition.drivercontrol(telop);
  return 0;
}
```

### resetPositionWithSensor has two signatures

The header declares a four-argument version; the implementation is a five-argument version. Only the five-argument one exists. This stays harmless as long as you follow the instruction already in the code and call `resetPositionFront/Back/Left/Right()` instead. Those wrappers pass the right arguments for you.

### The autotuner only handles distance

There is no turn autotuner. `evaluateTurnGains()` exists in `autotune.h` and works, but nothing calls it and no `runTurnAutoTune()` function exists to bind to a button. Tune `turn_*` by hand. See [Tuning]({{ site.baseurl }}/tuning#tuning-by-hand).

---

## The robot doesn't move at all

**Check the PID gains first.** All nine ship as `0`:

```cpp
double distance_kp = 0, distance_ki = 0, distance_kd = 0;
```

With `kp = 0` the PID output is zero, so `driveTo()` burns its whole time limit, commands nothing, and returns. This is the most common cause by a wide margin. Go to [Tuning]({{ site.baseurl }}/tuning).

If gains are set and it still won't move:

- Confirm ports and reversal flags in `robot-config.cpp` match the physical robot.
- Test in driver control first. If tank drive doesn't work, autonomous never will.
- Check that the program actually downloaded to **slot 1** and that you're running that slot.

## The build fails

**`Project name cannot contain whitespace`**
: The full path to the project folder has a space in it. `vex/mkenv.mk` derives the project name from the folder and rejects whitespace. Move the project somewhere like `C:\vex\RW-2.0`.

**`undefined reference to ...`**
: You called one of the [unimplemented functions](#functions-that-are-declared-but-not-implemented), or you declared a device in `robot-config.h` without defining it in `robot-config.cpp`.

**A device you added isn't recognized in another file**
: A device needs both the definition in `robot-config.cpp` and the `extern` declaration in `robot-config.h`.

To see the actual compiler commands, set `VERBOSE = 1` at the top of [`makefile`](https://github.com/Master101215/RW-2.0/blob/main/makefile).

## Movement problems

| Symptom | Likely cause |
|:--------|:-------------|
| Consistently travels the wrong distance by the same percentage | `wheel_distance_in` is wrong — measure the actual wheel diameter and gear ratio |
| Overshoots the target | `distance_kp` too high or `distance_kd` too low |
| Stops short and sits there | `distance_kp` too low; consider a small `distance_ki` |
| Oscillates around the target | `distance_kp` too high — lower it, then add `kd` |
| Veers off heading on straight drives | `heading_correction_*` untuned |
| Snakes side to side | `heading_correction_kp` too high |
| Vibrates or buzzes when stopped | `heading_correction_kp` too high, or set `heading_correction = false` |
| Wheelies or slips on launch | Lower `max_slew_accel_fwd` |
| Turns are fine at 90° but wrong at 15° | `turn_kd` too high, or add a small `turn_ki` |
| `boomerang` drifts or is inconsistent | Lower `chase_power`; check `dlead` is ≤ 0.6 |
| `boomerang` reaches the point at the wrong heading | Raise `dlead` (staying at or below 0.6) |
| Everything gets worse as the match goes on | Battery. Re-check gains on a fresh charge. |

{: .note }
> A move that hits its `time_limit_msec` stops wherever it happens to be, and everything after it runs from the wrong position. If a routine drifts progressively out of alignment, check whether an early move is timing out before blaming the later ones.

## Odometry problems

**X/Y stay at zero while the robot moves**
: Odometry never started, or the drive encoders aren't reading. Confirm the drive ports are right, and that you call `trackPosition()` at the top of `auton()` and `telop()`.

**Position drifts during a pure rotation**
: Tracker offsets are wrong. Spin the robot 360° in place by hand and watch X and Y. They should barely move. If they wander, fix `horizontal_tracker_dist_from_center` / `vertical_tracker_dist_from_center`. See [Configuration]({{ site.baseurl }}/configuration#tracker-geometry).

**Position is right at first, then progressively wrong**
: Normal drift. Use [position resets]({{ site.baseurl }}/api/odometry#position-resets) at wall-square points in the routine.

**A collision throws position off permanently**
: Without a horizontal tracking wheel, odometry can't see sideways motion. The encoders don't turn, so nothing registers. Add a horizontal tracker, or reset against a wall after contact.

**X and Y feel swapped**
: They aren't. The frame is compass style: **heading 0° faces +Y, and heading increases clockwise.** See [the coordinate frame]({{ site.baseurl }}/api/odometry#the-coordinate-frame).

**Position resets make things worse**
: Either the robot wasn't square to the wall, or the `*_offsetX`/`*_offsetY` values for that sensor are wrong. Check that the sensor points at a wall. The code discards a reading over 200 inches or below 0 without saying so, so a reset that "does nothing" usually means the sensor saw nothing.

## Controller and driver control

**The robot drives off on its own**
: Button Y is bound to the autotuner in `telop()`. Delete `controller_1.ButtonY.pressed(runDistanceAutoTune);` once tuning is done.

**The robot stopped responding after tuning**
: `runDistanceAutoTune()` ends in an intentional infinite loop. Restart the program.

**No terminal output from `logPosition()`**
: That task prints over the controller's USB connection. Connect the controller to the computer by cable.

**One side drives backward**
: Fix the reversal flag for those motors in `robot-config.cpp`.

**Controls feel wrong**
: Check `drive_scheme`. The default is `TANK` (Axis3 / Axis2); the alternatives are `ARCADE` and `SPLIT_ARCADE`. See [Configuration]({{ site.baseurl }}/configuration#drive-scheme).