---
layout: default
title: API Reference
nav_order: 5
has_children: true
permalink: /api/
---

# API Reference

Everything RW 2.0 gives you to call from `auton()`, split across three pages.
{: .fs-6 .fw-300 }

---

## [Motion]({{ site.baseurl }}/api/motion)

The seven movement primitives, plus the low-level chassis helpers.

| Function | Drives to |
|:---------|:----------|
| `driveTo` | A distance, in a straight line |
| `turnToAngle` | A field heading, turning in place |
| `swing` | A heading, pivoting around one side |
| `curveCircle` | A heading, along a fixed-radius arc |
| `turnToPoint` | Faces a field coordinate |
| `moveToPoint` | A field coordinate |
| `boomerang` | A field coordinate *and* a final heading |

Also covers [motion chaining]({{ site.baseurl }}/api/motion#motion-chaining), which runs commands back to back without stopping between them.

## [Odometry]({{ site.baseurl }}/api/odometry)

The coordinate frame, the four position-tracking modes, the `x_pos` / `y_pos` globals, and resetting position against a wall with distance sensors.

## [PID]({{ site.baseurl }}/api/pid)

The `PID` class under the motion code, and how to point it at your own arm, lift, or intake.

---

## What you don't call

[`include/motor-control.h`](https://github.com/Master101215/RW-2.0/blob/main/include/motor-control.h) declares a few functions that have no implementation. Calling them fails at link time. This reference skips them, so see [Troubleshooting]({{ site.baseurl }}/troubleshooting#functions-that-are-declared-but-not-implemented) for the list.