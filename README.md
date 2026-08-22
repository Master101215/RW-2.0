# RW-Template 2.0

An advanced **VEX V5 autonomous robotics template** focused on precise, algorithm-driven motion planning and control. Built for teams who want robust, adaptable autonomous routines across a wide range of sensor and drive configurations.

### 📚 **[Read the full documentation →](https://master101215.github.io/RW-2.0/)**

---

## 🚀 Features

### ✅ Universal PID Class
- Modular, reusable PID controller for all motion and heading tasks — and for your own subsystems.
- Integral clamping, integral-range gating, and a two-stage exit condition so movements settle instead of hunting.

### 📍 Odometry
- Tracks robot position using drivetrain encoders and/or tracking wheels for accurate field navigation.
- Runs continuously in a background thread, updating every 10 ms.

### 🧭 IMU Support
- Integrates the inertial sensor for heading correction and absolute orientation.

### 🔁 Tracking Wheel Flexibility
- Works with or without tracking wheels.
- Supports all tracking wheel configurations.
- Falls back to drivetrain encoders and IMU alone for basic pose estimation.

### 🎯 Turn to Face Point/Heading
- Turn to any absolute field heading, or to face any field coordinate.

### 🔄 Swing to Face Heading
- Swing turns for efficient, single-side turning.

### 🏹 Move to Point
- Navigate to a specific coordinate with continuous position and heading correction.
- Works consistently with or without tracking wheels.

### 📐 Curve Path Movement (`curveCircle`)
- Executes smooth, constant-radius arcs.
- Fully compatible with robots **not using tracking wheels or full odometry** — uses heading + distance PID only.

### 🏹 Move to Pose via Boomerang
- Boomerang controller for smooth, curved motion to a target *pose* — coordinate plus final heading.

### ⛓️ Motion Chaining
- Link motion commands so the robot rolls through instead of stopping between them.

### 🧭📍 Distance Resets
- Position resets against field walls using distance sensors.
- Use sensors on all sides, or just the specific sides you have.

### 🔧 Built-in PID Autotuner
- Press **Y** on the controller and a Twiddle optimizer tunes your distance gains automatically.

---

## 🛠️ Installation

1. **Install [Visual Studio Code](https://code.visualstudio.com/)**

2. **Install the VEX Robotics Extension in VS Code**
   - Open the Extensions view (`Ctrl/Cmd+Shift+X`)
   - Search for and install **"VEX Robotics Extension"** by VEX Robotics
   - It supplies the entire toolchain — you don't need to install clang or an ARM compiler separately

3. **Get the project**
   ```bash
   git clone https://github.com/Master101215/RW-2.0.git
   ```
   Or download the source ZIP and unzip it.

   > ⚠️ **The full path to the project folder must not contain spaces.** The build derives the project name from the folder name and fails with `Project name cannot contain whitespace`.

4. **Open the folder in VS Code** — select the folder containing `makefile`, not its parent.

5. **Build and download** with the VEX toolbar buttons. The program installs to **slot 1**.

---

## 📘 Usage Guide

### 1. Project Structure

**include/**
- `motor-control.h` — Drive and motion control declarations
- `pid.h` — Reusable PID controller class
- `robot-config.h` — Device and tuning parameter declarations, `DriveScheme` enum
- `utils.h` — Math and geometry helpers
- `vex.h` — VEX SDK includes and macros
- `autotune.h` — Twiddle-based PID autotuner
- `dsr.h` — Experimental wall-alignment routines (not included in the build)

**src/**
- `main.cpp` — Competition entry point, `auton()`, `telop()`, telemetry and odometry tasks
- `motor-control.cpp` — All motion control and odometry implementations
- `pid.cpp` — PID controller logic
- `robot-config.cpp` — **Your robot's ports, measurements, and tuning values**
- `utils.cpp` — Math and geometry implementations

**vex/**
- `mkenv.mk`, `mkrules.mk` — Toolchain configuration (don't edit)

### 2. Robot Configuration

Edit [`src/robot-config.cpp`](src/robot-config.cpp) to match your hardware, and add matching `extern` declarations to [`include/robot-config.h`](include/robot-config.h) for anything new.

- Set ports and gear cartridges for the drive motors, inertial sensor, and any tracking or distance sensors
- Set reversal flags so positive voltage drives each wheel forward
- Add your game mechanisms in the marked **subsystem devices** section

> ⚠️ Don't rename the built-in devices — the motion code references them by name. Change ports, not identifiers.

📖 [Full configuration reference →](https://master101215.github.io/RW-2.0/configuration)

### 3. Tuning Your Robot

In [`src/robot-config.cpp`](src/robot-config.cpp), find the **USER-CONFIGURABLE PARAMETERS** section:

- `distance_between_wheels` — inches between the left and right wheel centers
- `wheel_distance_in` — `(motor:wheel gear ratio) × wheel diameter × π`
- PID constants: `distance_*`, `turn_*`, `heading_correction_*`

> ⚠️ **All nine PID gains ship as `0`.** The robot will not move until you tune it.

**Using the autotuner:**

- The binding `controller_1.ButtonY.pressed(runDistanceAutoTune);` is **already present** in `telop()`
- Download and run the program in driver mode
- Put the robot in an open area with several feet of clear space front and back
- Press **Y** — the tuner drives back and forth, optimizing `distance_kp` and `distance_kd`
- When it shows `TUNE DONE` or `GOOD VALUES!`, read the values off the screen, **copy them into `robot-config.cpp` by hand**, and re-download

> The autotuner doesn't save anything, tunes distance only (turn gains are hand-tuned), and ends in an intentional infinite loop when finished. Delete the Button Y binding when you're done tuning.

**If using tracking wheels:**

- Set `using_horizontal_tracker = true` and/or `using_vertical_tracker = true`
- Configure tracker distances from center and diameters

**If using distance resets:**

- Set your ports on the prebuilt sensor declarations (do not change the names)
- Set each sensor's `offsetX` / `offsetY` in inches

📖 [Full tuning guide →](https://master101215.github.io/RW-2.0/tuning)

### 4. Autonomous Programming

Write your routine inside `auton()` in [`src/main.cpp`](src/main.cpp), below the task startup lines:

```cpp
void auton() {
  trackPosition();
  vex::task updater(updateTask);
  vex::task logposition(logPosition);
  update();
  inertial_sensor.setRotation(0, degrees);
  correct_angle = normalizeTarget(0);

  // Write your autonomous routine here.
  driveTo(24, 2000);
  turnToAngle(90, 1500);
  moveToPoint(24, 24, 1, 3000);
}
```

Available motion functions from `motor-control.h`:

```cpp
driveTo(distance_in, time_limit, exit, max_output);
turnToAngle(angle, time_limit, exit, max_output);
swing(angle, drive_direction, time_limit, exit, max_output);
curveCircle(result_angle, center_radius, time_limit, exit, max_output);
turnToPoint(x, y, dir, time_limit);
moveToPoint(x, y, dir, time_limit, exit, max_output, overturn);
boomerang(x, y, dir, a, dlead, time_limit, exit, max_output, overturn);
```

Pass `exit = false` to chain a movement into the next one without stopping. The last command in a chain must use `exit = true`.

> The coordinate frame is compass-style: **heading 0° faces +Y and heading increases clockwise.**

📖 [Full motion API →](https://master101215.github.io/RW-2.0/api/motion)

### 5. Driver Control

Edit `telop()` in [`src/main.cpp`](src/main.cpp).

The drive scheme is selected by `drive_scheme` in `robot-config.cpp`:

| Value | Controls |
|-------|----------|
| `TANK` (default) | Axis3 left side, Axis2 right side |
| `ARCADE` | Axis3 forward, Axis4 turn |
| `SPLIT_ARCADE` | Axis3 forward, Axis1 turn |

Add mechanism button bindings inside the `while (true)` loop, where the template marks the spot.

### 6. Competition Setup

`main()` constructs the competition object and registers the two callbacks:

```cpp
int main() {
  competition Competition = competition();
  Competition.autonomous(auton);
  Competition.drivercontrol(telop);
  return 0;
}
```

`auton()` runs during the autonomous period, `telop()` during driver control. There is no `pre_auton` callback — if you need startup work such as IMU calibration, add it to `main()` before the competition object is constructed.

### 7. Tips for Success

- Read the comments in each file — they clarify how each function and variable works
- Get `wheel_distance_in` and `distance_between_wheels` right *before* tuning PID; bad geometry looks exactly like bad gains
- Test motion functions individually before combining them
- Use position resets to correct odometry drift mid-routine
- Re-check your tuning on a fully charged battery
- Don't hesitate to reach out to the VEX community for help

### 8. Where to Start

1. Set ports and devices in [`src/robot-config.cpp`](src/robot-config.cpp) and [`include/robot-config.h`](include/robot-config.h)
2. Enter chassis measurements in [`src/robot-config.cpp`](src/robot-config.cpp)
3. Confirm movement and controls in driver control
4. Run the autotuner, then hand-tune turn and heading gains
5. Create and test simple autonomous routines
6. Expand with more complex logic and paths as you grow confident

---

## ⚠️ Known Limitations

- Several functions are declared in `motor-control.h` but not implemented (`wallDrive*`, `driveStraight`, `waitForBlock`) — calling them fails at link time
- In the shipped config, `left_sensor` and `inertial_sensor` are both on `PORT9`
- `include/dsr.h` references undeclared sensors and is not part of the build

📖 [Full troubleshooting guide →](https://master101215.github.io/RW-2.0/troubleshooting)

---

This template is **competition-ready** and provides a strong foundation for building reliable, high-performance autonomous routines using proven robotics algorithms, whether or not your robot uses advanced odometry.

Derived from [richardbwang/RW-Template](https://github.com/richardbwang/RW-Template).

If you find this template useful, please star this repository. ⭐
