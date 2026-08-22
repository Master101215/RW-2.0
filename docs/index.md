---
layout: default
title: Home
nav_order: 1
description: "RW 2.0 — a VEX V5 autonomous template with odometry, PID motion control, and boomerang pathing."
permalink: /
---

# RW 2.0
{: .fs-9 }

A VEX V5 autonomous template. The motion functions run whether or not you have tracking wheels, and everything sits on one reusable PID class.
{: .fs-6 .fw-300 }

[Get started]({{ site.baseurl }}/getting-started){: .btn .btn-primary .fs-5 .mb-4 .mb-md-0 .mr-2 }
[View on GitHub](https://github.com/Master101215/RW-2.0){: .btn .fs-5 .mb-4 .mb-md-0 }

---

## What this is

RW 2.0 is a starting point for a VEX V5 competition program. You get a working chassis, a background odometry thread, and seven motion functions to call from your autonomous routine: `driveTo`, `turnToAngle`, `swing`, `curveCircle`, `turnToPoint`, `moveToPoint`, and `boomerang`.

**Odometry is optional** by design. With horizontal and vertical tracking wheels, the template reads both. With one, it reads that wheel plus the drivetrain encoders. With none, it falls back to the encoders and the inertial sensor, and the motion functions still work.

You write your routine in `auton()` and your driver controls in `telop()`, both in [`src/main.cpp`](https://github.com/Master101215/RW-2.0/blob/main/src/main.cpp). The tunables all live in [`src/robot-config.cpp`](https://github.com/Master101215/RW-2.0/blob/main/src/robot-config.cpp).

## Features

**Universal PID class**
: One controller ([`include/pid.h`]({{ site.baseurl }}/api/pid)) runs the motion and heading work, and you can point it at your own subsystems. It clamps the integral, gates it by error range, and exits on a two-stage condition so movements settle instead of hunting.

**Odometry that adapts to your hardware**
: Two booleans pick one of four tracking modes at startup. A background thread updates position at 100 Hz.

**IMU heading correction**
: A correction PID holds heading during straight moves and while the robot sits still, so a routine doesn't drift sideways as it goes.

**Point and pose navigation**
: `turnToPoint` and `moveToPoint` navigate to field coordinates. `boomerang` drives to a *pose*, meaning a coordinate plus a final heading, along a curved approach.

**Curved paths without odometry**
: `curveCircle` runs a constant-radius arc on heading and distance PID alone, so it works on a robot with no tracking wheels.

**Motion chaining**
: Pass `exit = false` and the robot rolls into the next command instead of stopping. Routines built this way flow instead of stuttering.

**Distance-sensor position resets**
: Square up against a wall and call `resetPositionFront()` (or back, left, right) to snap odometry back to the truth mid-routine. Use however many sensors you have.

**Built-in PID autotuner**
: Press **Y** on the controller and a Twiddle optimizer tunes the distance gains for you.

## Where to go next

| Page | What's in it |
|:-----|:-------------|
| [Getting Started]({{ site.baseurl }}/getting-started) | Install the toolchain, build, download to the brain, first drive test |
| [Configuration]({{ site.baseurl }}/configuration) | Every port, measurement, and tunable in `robot-config.cpp` |
| [Tuning]({{ site.baseurl }}/tuning) | The autotuner, then hand-tuning turns and heading correction |
| [API Reference]({{ site.baseurl }}/api/) | Motion functions, odometry, and the PID class |
| [Troubleshooting]({{ site.baseurl }}/troubleshooting) | Known limitations and common failure modes |

## Before your first run

{: .warning }
> **Every PID gain ships as `0`.** The template will not move the robot until you tune it. Set your ports and chassis measurements first, then work through [Tuning]({{ site.baseurl }}/tuning). See [Troubleshooting]({{ site.baseurl }}/troubleshooting) for the other known rough edges in the shipped config.

## Credits

RW 2.0 is derived from [richardbwang/RW-Template](https://github.com/richardbwang/RW-Template).