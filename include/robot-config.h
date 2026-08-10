#pragma once
#include "vex.h"
using namespace vex;

// ============================================================================
// DEVICE DECLARATIONS
// Format: extern deviceType deviceName;
// Define the actual device (with port number) in robot-config.cpp
// ============================================================================

extern brain Brain;
extern controller controller_1;

// ── Drivetrain ────────────────────────────────────────────────────────────
extern motor left_chassis1;
extern motor left_chassis2;
extern motor_group left_chassis;
extern motor right_chassis1;
extern motor right_chassis2;
extern motor_group right_chassis;
extern drivetrain Drivetrain;

extern inertial inertial_sensor;

// ── Tracking wheels (odometry) ───────────────────────────────────────────
// Set to any unused port if you are not using tracking wheels.
extern rotation horizontal_tracker;
extern rotation vertical_tracker;

// ── Distance sensors (optional, used for position reset against walls) ──
// Uncomment and wire up in robot-config.cpp if used.
extern distance front_sensor;
extern distance left_sensor;
extern distance right_sensor;
extern distance back_sensor;

// ── Subsystem devices ─────────────────────────────────────────────────────
// Add your game-specific mechanisms here, e.g.:
// extern motor arm;
// extern rotation arm_rotation;
// extern motor_group lift;
// extern motor intake;

// ============================================================================
// DISTANCE SENSOR OFFSETS (only needed if using distance-sensor position reset)
// ============================================================================
extern double front_sensor_offsetX;
extern double front_sensor_offsetY;
extern double left_sensor_offsetX;
extern double left_sensor_offsetY;
extern double right_sensor_offsetX;
extern double right_sensor_offsetY;
extern double back_sensor_offsetX;
extern double back_sensor_offsetY;
extern double distance_between_sensors;

// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================
extern double distance_between_wheels;
extern double wheel_distance_in;
extern double distance_kp, distance_ki, distance_kd;
extern double turn_kp, turn_ki, turn_kd;
extern double heading_correction_kp, heading_correction_ki, heading_correction_kd;

extern bool using_horizontal_tracker;
extern bool using_vertical_tracker;
extern double horizontal_tracker_dist_from_center;
extern double vertical_tracker_dist_from_center;
extern double horizontal_tracker_diameter;
extern double vertical_tracker_diameter;

extern bool heading_correction;
extern bool dir_change_start;
extern bool dir_change_end;
extern double min_output;
extern double max_slew_accel_fwd;
extern double max_slew_decel_fwd;
extern double max_slew_accel_rev;
extern double max_slew_decel_rev;
extern double chase_power;

// ============================================================================
// DRIVE SCHEME
// ============================================================================
// Controls how the controller axes map to chassis motors during driver control.
//
//   TANK         — Left Axis3 -> left side,  Right Axis2 -> right side.
//   ARCADE       — Left Axis3 = forward/back,  Left Axis4 = turn.
//   SPLIT_ARCADE — Left Axis3 = forward/back,  Right Axis1 = turn.
//
// Change drive_scheme in robot-config.cpp to switch between them.

enum DriveScheme {
    TANK,
    ARCADE,
    SPLIT_ARCADE
};

extern DriveScheme drive_scheme;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
