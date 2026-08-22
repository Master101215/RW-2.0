---
layout: default
title: PID
parent: API Reference
nav_order: 3
---

# PID
{: .no_toc }

The controller under every motion function, declared in [`include/pid.h`](https://github.com/Master101215/RW-2.0/blob/main/include/pid.h). You can point it at your own mechanisms too.
{: .fs-6 .fw-300 }

<details open markdown="block">
  <summary>On this page</summary>
  {: .text-delta }
- TOC
{:toc}
</details>

---

## Basic use

```cpp
PID pid_arm(0.5, 0.0, 2.0);        // kp, ki, kd
pid_arm.setTarget(450);            // degrees

while (!pid_arm.targetArrived()) {
  double output = pid_arm.update(arm.position(degrees));
  arm.spin(forward, output, volt);
  wait(10, msec);
}
arm.stop(hold);
```

`update()` takes the current sensor reading, computes the output, and returns it. Call it on a fixed interval. The motion code uses 10 ms.

## Constructor

```cpp
PID(double new_kp, double new_ki, double new_kd);
```

The constructor fills in defaults for the rest:

| Setting | Default |
|:--------|:--------|
| `small_error_tolerance` | 1 |
| `big_error_tolerance` | 3 |
| `small_error_duration` | 100 ms |
| `big_error_duration` | 500 ms |
| `integral_max` | 500 |
| `integral_range` | 0 (disabled) |
| `arrive` | `true` |

## The exit condition

This is the part that makes movements settle instead of hunting.

The controller reports "arrived" through **two independent checks**, and either one triggers it:

| Check | Condition |
|:------|:----------|
| Tight | error within `small_error_tolerance` **and** derivative within `derivative_tolerance`, held for `small_error_duration` |
| Loose | error within `big_error_tolerance` **and** derivative within `derivative_tolerance`, held for `big_error_duration` |

Each check has its own timer, and the timer resets as soon as the condition breaks. The robot has to hold the condition. A fast pass through tolerance fails the derivative test, so it doesn't count.

The pairing does two jobs. The tight check catches a clean, fast settle. The loose one covers the case where the robot lands close but can't close the last fraction of an inch, whether from friction, a dead spot, or a low `kp`. It exits after half a second instead of grinding against the timeout.

```cpp
pid.setSmallBigErrorTolerance(0.5, 1.5);   // tight, loose
pid.setSmallBigErrorDuration(50, 250);     // tight ms, loose ms
pid.setDerivativeTolerance(5);             // "slow enough" threshold
```

`driveTo` uses those values internally.

### Disabling the exit condition

```cpp
pid.setArrive(false);
```

`targetArrived()` then never returns `true`. Use it for a controller that runs continuously. The heading-correction PID inside the motion functions works this way, since it has no destination to reach.

## Integral handling

The integral term causes the most trouble in practice, so it gets three guards.

**Range gating** ignores the integral while error is large:

```cpp
pid.setIntegralRange(1);
```

Above an error of 1, `sum_error` stays at zero. That stops the integral winding up during a long approach and then slamming the output on arrival. Set it to `0` to disable gating.

**Clamping** caps the integral's contribution:

```cpp
pid.setIntegralMax(3);
```

Limits `ki * sum_error` to this value. Set to `0` to disable.

**Sign reset** happens automatically, with no setting. The controller zeroes `sum_error` whenever the error changes sign (the robot overshot) or falls within `small_error_tolerance`. Accumulated integral does its worst damage at both of those moments.

{: .tip }
> Start with `ki = 0`. Most VEX drive movements never need integral at all, and it's the fastest way to make a well-tuned P/D controller oscillate. Reach for it only if the robot reliably stops *short* of the target and stays there.

## Method reference

| Method | Purpose |
|:-------|:--------|
| `setCoefficient(kp, ki, kd)` | Change gains on a live controller |
| `setTarget(target)` | Set the setpoint |
| `setIntegralMax(max)` | Cap the integral contribution; `0` disables |
| `setIntegralRange(range)` | Ignore integral outside this error band; `0` disables |
| `setSmallBigErrorTolerance(small, big)` | Error bands for the two exit checks |
| `setSmallBigErrorDuration(small, big)` | How long each band must hold, in ms |
| `setDerivativeTolerance(tol)` | Max rate of change still counted as settled |
| `setArrive(bool)` | `false` disables the exit condition entirely |
| `clearSumError()` | Zero the accumulated integral |
| `targetArrived()` | Has the exit condition been met? |
| `update(input)` | Step the controller; returns the new output |
| `getOutput()` | Last computed output, without stepping |
| `getI()` | Returns the `ki` coefficient |
| `sign(number)` | −1, 0, or 1 |

{: .note }
> `update()` skips the derivative on its very first call, using the current error as the previous error. Without that, the first sample would produce a huge phantom derivative spike.

## Running a subsystem on PID

The pattern below is the one commented into [`src/main.cpp`](https://github.com/Master101215/RW-2.0/blob/main/src/main.cpp). Add one block per mechanism.

```cpp
double arm_pid_target = 0;
PID pidArm(0.5, 0.0, 2.0);

void armPID(double target) {
  pidArm.setTarget(target);
  pidArm.setIntegralMax(0);
  pidArm.setIntegralRange(1);
  pidArm.setSmallBigErrorTolerance(1, 1);
  pidArm.setSmallBigErrorDuration(0, 0);
  pidArm.setDerivativeTolerance(100);
  pidArm.setArrive(true);
  arm.spin(fwd, pidArm.update(arm.position(deg)), volt);
  // or read a rotation sensor instead: arm_rotation.position(deg)
}

int armPIDLoop() {
  while (true) {
    armPID(arm_pid_target);
    wait(10, msec);
  }
  return 0;
}
```

Start it as a task in `telop()` and `auton()`:

```cpp
task armTask(armPIDLoop);
```

Then move the mechanism by assigning the target from anywhere. The task handles the rest:

```cpp
arm_pid_target = 450;    // arm drives to 450° and holds there
```

{: .tip }
> Because the loop runs forever and re-reads `arm_pid_target` every iteration, the mechanism actively *holds* position against gravity rather than just reaching it. For an arm or lift this is usually what you want. Set the target rather than commanding the motor directly, or the two will fight.