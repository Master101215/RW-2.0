---
layout: default
title: Configuration
nav_order: 3
---

# Configuration
{: .no_toc }

You fit the template to your robot in [`src/robot-config.cpp`](https://github.com/Master101215/RW-2.0/blob/main/src/robot-config.cpp), with matching `extern` declarations in [`include/robot-config.h`](https://github.com/Master101215/RW-2.0/blob/main/include/robot-config.h). This page walks that file top to bottom.
{: .fs-6 .fw-300 }

<details open markdown="block">
  <summary>On this page</summary>
  {: .text-delta }
- TOC
{:toc}
</details>

---

## Drivetrain

```cpp
motor left_chassis1  = motor(PORT1, ratio6_1, true);
motor left_chassis2  = motor(PORT2, ratio6_1, true);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2);

motor right_chassis1 = motor(PORT3, ratio6_1, false);
motor right_chassis2 = motor(PORT4, ratio6_1, false);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2);
```

The constructor is `motor(port, gearSetting, reversed)`.

| Cartridge | Constant | RPM |
|:----------|:---------|:----|
| Red | `ratio36_1` | 100 |
| Green | `ratio18_1` | 200 |
| Blue (also 5.5 W) | `ratio6_1` | 600 |

The left side ships reversed and the right side doesn't, since the two sides face opposite directions on a normal chassis. The flags themselves don't matter. What matters is the result: **set the reversal so positive voltage moves that wheel forward.** Test it in driver control before you trust an autonomous.

### More or fewer motors

The motion code never touches individual motors. It works through `left_chassis` and `right_chassis` only, so to run a six-motor drive you add the motors and put them in the groups:

```cpp
motor left_chassis3 = motor(PORT5, ratio6_1, true);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);
```

Add a matching `extern motor left_chassis3;` in `robot-config.h`.

### Drivetrain object

```cpp
drivetrain Drivetrain = drivetrain(left_chassis, right_chassis, 3.17, 6.57, 1.5, inches);
```

Arguments are wheel travel, track width, wheelbase, and units. This object exists for VEXcode compatibility. The RW 2.0 motion functions read `wheel_distance_in` and `distance_between_wheels` below, so **those two are the numbers that actually move your robot**.

## Sensors

```cpp
inertial inertial_sensor = inertial(PORT9);

rotation horizontal_tracker = rotation(PORT10, true);
rotation vertical_tracker   = rotation(PORT16, true);

distance front_sensor = distance(PORT13);
distance left_sensor  = distance(PORT9);
distance right_sensor = distance(PORT11);
distance back_sensor  = distance(PORT20);
```

{: .warning }
> **Do not rename these devices.** The motion code in `motor-control.cpp` refers to them by name directly. Change the port numbers, not the identifiers.

{: .warning }
> `left_sensor` and `inertial_sensor` are both on `PORT9` in the shipped config. Move one before using left-side position resets.

The inertial sensor is required. Heading correction, turning, and all four odometry modes depend on it.

The rotation and distance sensors are optional. Without tracking wheels, leave `horizontal_tracker` and `vertical_tracker` pointed at any unused port and set the `using_*_tracker` flags to `false`. The objects get constructed and never read. The same goes for the distance sensors if you never call a position reset.

## Adding subsystems

Declare your mechanism devices alongside the drivetrain:

```cpp
// in src/robot-config.cpp
motor arm = motor(PORT6, ratio36_1, true);
rotation arm_rotation = rotation(PORT7, false);
motor intake = motor(PORT8, ratio6_1, true);
```

```cpp
// in include/robot-config.h
extern motor arm;
extern rotation arm_rotation;
extern motor intake;
```

Both files have a marked **subsystem devices** section for this. To run a mechanism under PID control, see the [subsystem PID recipe]({{ site.baseurl }}/api/pid#running-a-subsystem-on-pid).

## Required measurements

These two numbers sit under every distance-based movement and under odometry itself. If they're wrong, PID tuning won't recover your accuracy.

```cpp
double distance_between_wheels = 11.2;
double wheel_distance_in = (48.0 / 72.0) * 3.1 * M_PI;
```

**`distance_between_wheels`** is the distance in inches from the center of the left wheels to the center of the right wheels. Measure it. Don't estimate from the CAD.

**`wheel_distance_in`** is how far the robot travels per *motor* revolution, in inches. The formula:

```
(motor gear teeth ÷ wheel gear teeth) × wheel diameter × π
```

The shipped value works through as:

| Term | Value | Meaning |
|:-----|:------|:--------|
| `48.0 / 72.0` | 0.667 | 48-tooth motor gear driving a 72-tooth wheel gear |
| `3.1` | 3.1 in | measured wheel diameter |
| `M_PI` | π | circumference |
| **Result** | **6.49 in** | travel per motor revolution |

{: .tip }
> Use the *measured* diameter of your wheels, not the nominal one. A nominally 3.25″ omni that has worn to 3.1″ will make every drive command 5% short, which looks exactly like an untuned PID.
>
> To verify: set `distance_kp` and friends, then run `driveTo(24, 3000)` and measure what the robot actually traveled. If it consistently goes 22.8″ when asked for 24″, your `wheel_distance_in` is 5% too small — scale it up rather than cranking `kp`.

On a direct drive, where the motor shares a shaft with the wheel, the ratio is `1.0`.

## PID gains

```cpp
double distance_kp = 0, distance_ki = 0, distance_kd = 0;
double turn_kp = 0, turn_ki = 0, turn_kd = 0;
double heading_correction_kp = 0, heading_correction_ki = 0, heading_correction_kd = 0;
```

| Group | Controls |
|:------|:---------|
| `distance_*` | Linear PID for straight driving — used by `driveTo`, `moveToPoint`, `boomerang`, `curveCircle` |
| `turn_*` | Turning in place — used by `turnToAngle`, `turnToPoint`, `swing` |
| `heading_correction_*` | Keeping the robot pointed straight *during* a linear move, and holding heading while stationary |

{: .warning }
> These are placeholders. All nine ship as `0` and the robot will not move until you set them. See [Tuning]({{ site.baseurl }}/tuning).

## Tracking wheels

```cpp
bool using_horizontal_tracker = false;
bool using_vertical_tracker = false;
```

These two booleans select the odometry mode at startup:

| `using_horizontal_tracker` | `using_vertical_tracker` | Mode used |
|:--------------------------|:-------------------------|:----------|
| `true` | `true` | `trackXYOdomWheel` — both trackers + IMU |
| `true` | `false` | `trackXOdomWheel` — horizontal tracker + drive encoders + IMU |
| `false` | `true` | `trackYOdomWheel` — vertical tracker + IMU |
| `false` | `false` | `trackNoOdomWheel` — drive encoders + IMU only |

You don't call these functions yourself. `trackPosition()` picks one and starts it as a thread. See [Odometry]({{ site.baseurl }}/api/odometry) for what each mode can and can't measure.

### Tracker geometry

```cpp
double horizontal_tracker_dist_from_center = 0;
double vertical_tracker_dist_from_center = 0;
double horizontal_tracker_diameter = 2.75;  // in
double vertical_tracker_diameter = 2;       // in
```

Skip these if both `using_*_tracker` flags are `false`.

Sign conventions are top-down, with the robot facing **up**:

| Parameter | Measures | Positive means |
|:----------|:---------|:---------------|
| `horizontal_tracker_dist_from_center` | Vertical (fore/aft) offset of the *horizontal* wheel from robot center | **behind** center |
| `vertical_tracker_dist_from_center` | Horizontal (left/right) offset of the *vertical* wheel from robot center | **right** of center |

These offsets matter because a tracking wheel that sits off the center of rotation records travel during a pure turn. The odometry math subtracts that out, but only if the offset is accurate. Measure to the *center of the tracking wheel's contact patch*.

The diameters are the real tracking wheel diameters. Measure them, and account for wear.

## Distance sensor offsets

You need these only if you call the position-reset functions.

```cpp
double front_sensor_offsetX = 0.0;
double front_sensor_offsetY = 0.0;

double left_sensor_offsetX = 0.0;
double left_sensor_offsetY = 0.0;

double right_sensor_offsetX = 0.0;
double right_sensor_offsetY = 0.0;

double back_sensor_offsetX = 0.0;
double back_sensor_offsetY = 0.0;

double distance_between_sensors = 0.0;
```

Each pair gives where that sensor sits relative to the robot's center, in inches, in the robot's own frame. The comment in the file shows the shape of it. A sensor sitting dead center left to right, mounted 6.5″ toward the front:

```cpp
double front_sensor_offsetX = 0.0;
double front_sensor_offsetY = 6.5;
```

The reset math uses these to convert "the sensor is 14 inches from the wall" into "the robot's center is at Y = 55.75". Get them wrong and each reset adds that much error. See [Position resets]({{ site.baseurl }}/api/odometry#position-resets).

## Advanced tuning

```cpp
bool heading_correction = true;

bool dir_change_start = true;
bool dir_change_end = true;

double min_output = 5;

double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;

double chase_power = 2.5;
```

| Setting | Default | What it does |
|:--------|:--------|:-------------|
| `heading_correction` | `true` | Hold the target heading with PID while the robot is stationary. Turn off if a stationary robot buzzes or fights itself. |
| `dir_change_start` | `true` | Softer acceleration at the start of a move, anticipating a direction change. `true` = accuracy and smoothness, `false` = speed. |
| `dir_change_end` | `true` | Same, for the end of a move. |
| `min_output` | `5` V | Floor on output voltage while chaining movements, so a chained move doesn't stall out at low error. |
| `max_slew_accel_fwd` | `24` | Max volt change per 10 ms while speeding up, driving forward. |
| `max_slew_decel_fwd` | `24` | Max volt change per 10 ms while slowing down, driving forward. |
| `max_slew_accel_rev` | `24` | Same, accelerating in reverse. |
| `max_slew_decel_rev` | `24` | Same, decelerating in reverse. |
| `chase_power` | `2.5` | Limits how hard `boomerang` cuts a corner, to stop the drive from slipping. Lower = less drift and more consistency, higher = more speed. |

The four `max_slew_*` values rate-limit the motors. At `24`, output can swing a full 12 V in about 5 ms, which is no limit at all in practice. Lower them, say to `6`, if your robot wheelies, tips, or slips on hard starts. Acceleration and deceleration are separate for each direction, so you can allow a quick launch and still keep a gentle stop.

## Drive scheme

```cpp
DriveScheme drive_scheme = TANK;
```

| Value | Left side | Right side |
|:------|:----------|:-----------|
| `TANK` | Axis 3 (left stick, vertical) | Axis 2 (right stick, vertical) |
| `ARCADE` | Axis 3 + Axis 4 | Axis 3 − Axis 4 (both on the left stick) |
| `SPLIT_ARCADE` | Axis 3 + Axis 1 | Axis 3 − Axis 1 (left stick drives, right stick turns) |

The `telop()` loop reads this every iteration, clamps each side to ±100%, and applies it with `setVelocity` / `spin` in percent units.

### Button bindings

Add your mechanism controls inside the `telop()` loop in [`src/main.cpp`](https://github.com/Master101215/RW-2.0/blob/main/src/main.cpp), where the template already marks the spot:

```cpp
while (true) {
  // ... drive scheme handling ...

  if (controller_1.ButtonR1.pressing()) {
    intake.spin(forward, 12, volt);
  } else if (controller_1.ButtonR2.pressing()) {
    intake.spin(reverse, 12, volt);
  } else {
    intake.stop(coast);
  }

  this_thread::sleep_for(20);
}
```

**Button Y is already taken.** It's bound to the distance autotuner at the bottom of the loop:

```cpp
controller_1.ButtonY.pressed(runDistanceAutoTune);
```

Remove that line once you've finished tuning, so a stray press during a match doesn't launch a tuning run. See [Tuning]({{ site.baseurl }}/tuning#the-autotuner).

{: .note }
> Use `.pressing()` for held actions (checked every loop) and `.pressed(callback)` to register a one-shot handler. The template's `.pressed()` call sits inside the loop, so it re-registers every 20 ms — harmless, but if you add your own `.pressed()` bindings, put them *above* the `while (true)` loop.