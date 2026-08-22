---
layout: default
title: Getting Started
nav_order: 2
---

# Getting Started
{: .no_toc }

From a fresh machine to a robot driving under your control.
{: .fs-6 .fw-300 }

<details open markdown="block">
  <summary>On this page</summary>
  {: .text-delta }
- TOC
{:toc}
</details>

---

## 1. Install the toolchain

RW 2.0 builds with the standard VEXcode toolchain, which ships inside the VS Code extension. You don't need a separate clang or ARM toolchain.

1. Install [Visual Studio Code](https://code.visualstudio.com/).
2. Open the Extensions view (<kbd>Ctrl</kbd>/<kbd>Cmd</kbd> + <kbd>Shift</kbd> + <kbd>X</kbd>).
3. Search for **VEX Robotics** (publisher: VEX Robotics) and install it.
4. Let the extension finish downloading its SDK on first launch.

The project pins SDK `V5_20240802_15_00_00` in [`.vscode/vex_project_settings.json`](https://github.com/Master101215/RW-2.0/blob/main/.vscode/vex_project_settings.json), and the extension fetches that version for you.

## 2. Get the project

```bash
git clone https://github.com/Master101215/RW-2.0.git
```

Or download the source ZIP from the repository and unzip it.

{: .warning }
> **The full path to the project folder must not contain spaces.**
>
> [`vex/mkenv.mk`](https://github.com/Master101215/RW-2.0/blob/main/vex/mkenv.mk) derives the project name from the folder name and hard-fails the build with `Project name cannot contain whitespace` if it finds any. A path like `C:\Users\you\Coding Projects\RW-2.0` will not build from the command line. Move it somewhere like `C:\vex\RW-2.0` first.

Open the folder in VS Code with **File → Open Folder**. Select the folder that contains `makefile`, rather than its parent.

## 3. Build and download

With the VEX extension installed, a VEX toolbar appears in VS Code:

| Button | What it does |
|:-------|:-------------|
| **Build** | Compiles to `build/RW V2.elf`, then converts to `build/RW V2.bin` |
| **Download** | Builds, then sends the `.bin` to the connected brain |
| **Run** | Downloads and immediately starts the program |

Connect the brain over USB before you download, or connect a controller that's paired to the brain. The program installs to **slot 1**, which `"slot": 1` in `.vscode/vex_project_settings.json` controls.

### Building from the command line

The VEX buttons just invoke `make`, so you can do it yourself:

```bash
make          # build
make clean    # remove the build/ directory
```

This needs `VEX_SDK_PATH` pointing at the SDK the extension downloaded (it defaults to `${HOME}/sdk`). To see the full compiler invocations, change the first real line of [`makefile`](https://github.com/Master101215/RW-2.0/blob/main/makefile):

```make
VERBOSE = 1
```

## 4. Set your ports

Open [`src/robot-config.cpp`](https://github.com/Master101215/RW-2.0/blob/main/src/robot-config.cpp) and change the port numbers and gear cartridges to match your robot. At minimum:

```cpp
motor left_chassis1  = motor(PORT1, ratio6_1, true);
motor left_chassis2  = motor(PORT2, ratio6_1, true);
motor right_chassis1 = motor(PORT3, ratio6_1, false);
motor right_chassis2 = motor(PORT4, ratio6_1, false);

inertial inertial_sensor = inertial(PORT9);
```

The `true`/`false` at the end is the reversal flag. Set it so **positive voltage drives that wheel forward** on both sides.

{: .warning }
> In the shipped config, `left_sensor` is on `PORT9`, the same port as `inertial_sensor`. Change one of them before you use left-side distance resets. See [Troubleshooting]({{ site.baseurl }}/troubleshooting).

Then set your chassis measurements in the same file:

```cpp
double distance_between_wheels = 11.2;                    // inches, left wheel center to right
double wheel_distance_in = (48.0 / 72.0) * 3.1 * M_PI;    // gear ratio × wheel diameter × π
```

Getting these two numbers right matters more than any PID gain. See [Configuration]({{ site.baseurl }}/configuration#required-measurements).

## 5. First drive test

Download the program and put the brain in driver control.

The template starts three background tasks as soon as `telop()` runs, so the screens should come alive right away:

**On the controller screen**

```
Hdg: 0.0
X:0.0 Y:0.0
Batt: 87%
```

**On the brain screen**

```
ODOM  x:0.0 y:0.0 h:0.0
                                    RW V2
```

Now drive it. The default scheme is tank: left stick (Axis3) runs the left side, right stick (Axis2) runs the right side. For arcade or split arcade, change `drive_scheme` in `robot-config.cpp`.

Push the robot forward by hand and watch `Y` climb, then spin it and watch `Hdg` change. If the numbers move sensibly, odometry is alive. If `X`/`Y` sit at zero while you push, check that your motors and inertial sensor are on the ports you configured.

{: .note }
> The telemetry comes from `updateTask()` in [`src/main.cpp`](https://github.com/Master101215/RW-2.0/blob/main/src/main.cpp), which refreshes every 100 ms. There is also a `logPosition()` task printing X/Y/heading to the VS Code terminal — that one needs the controller connected by cable.

## 6. Tune before writing autonomous

{: .warning }
> **Every PID gain in `robot-config.cpp` ships as `0`.** Until you tune, `driveTo()` and every other motion function will run their timeout and move the robot nowhere.

```cpp
double distance_kp = 0, distance_ki = 0, distance_kd = 0;
double turn_kp = 0, turn_ki = 0, turn_kd = 0;
double heading_correction_kp = 0, heading_correction_ki = 0, heading_correction_kd = 0;
```

Head to [Tuning]({{ site.baseurl }}/tuning) next. The autotuner handles `distance_kp` and `distance_kd` for you in a couple of minutes, and you hand-tune the turn and heading gains after that.

## 7. Write your routine

Autonomous goes in `auton()` in `src/main.cpp`, below the task startup:

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

See the [Motion API]({{ site.baseurl }}/api/motion) for the full command list, and [Odometry]({{ site.baseurl }}/api/odometry) for how the coordinate frame is oriented.