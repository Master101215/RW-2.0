#include "vex.h"
#include "robot-config.h"

using namespace vex;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller controller_1 = controller(primary);

// ============================================================================
// DRIVETRAIN
// Format: motor(port, gearSetting, reversed)
// gearSetting: ratio36_1 (red), ratio18_1 (green), ratio6_1 (blue/5.5w)
// Reverse motors as needed so that positive voltage spins the wheel forward.
// ============================================================================
motor left_chassis1  = motor(PORT1, ratio6_1, true);
motor left_chassis2  = motor(PORT2, ratio6_1, true);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2);

motor right_chassis1 = motor(PORT3, ratio6_1, false);
motor right_chassis2 = motor(PORT4, ratio6_1, false);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2);

// drivetrain(left, right, wheelTravel_in, trackWidth_in, wheelbase_in, units)
drivetrain Drivetrain = drivetrain(left_chassis, right_chassis, 3.17, 6.57, 1.5, inches);

inertial inertial_sensor = inertial(PORT9);

// ── Tracking wheels ──────────────────────────────────────────────────────
// Format: rotation(port, reversed)
// Set to any unused port if you don't use tracking wheels.
rotation horizontal_tracker = rotation(PORT10, true);
rotation vertical_tracker   = rotation(PORT16, true);

// ── Distance sensors ──────────────────────────────────────────
distance front_sensor = distance(PORT13);
distance left_sensor = distance(PORT9);
distance right_sensor = distance(PORT11);
distance back_sensor = distance(PORT20);

// ============================================================================
// SUBSYSTEM DEVICES
// Add your game-specific mechanisms below, following the same pattern as the
// drivetrain above, e.g.:
//
// motor arm = motor(PORT6, ratio36_1, true);
// rotation arm_rotation = rotation(PORT7, false);
// motor_group lift(left_lift, right_lift);
// motor intake = motor(PORT8, ratio6_1, true);
// ============================================================================


// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
double distance_between_wheels = 11.2;

// motor to wheel gear ratio * wheel diameter (in inches) * pi
double wheel_distance_in = (48.0 / 72.0) * 3.1 * M_PI;

// PID Constants for movement
// distance_* : Linear PID for straight driving
// turn_*     : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
// NOTE: these are placeholder values — run the auto-tuner or hand-tune before use.
double distance_kp = 0, distance_ki = 0, distance_kd = 0;
double turn_kp = 0, turn_ki = 0, turn_kd = 0;
double heading_correction_kp = 0, heading_correction_ki = 0, heading_correction_kd = 0;

// Enable or disable the use of tracking wheels
bool using_horizontal_tracker = false;
bool using_vertical_tracker = false;

// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
// Perspective is top-down, with the robot facing "up" (vertical).
// Vertical distance from bot center to horizontal tracking wheel (in, + = behind center)
double horizontal_tracker_dist_from_center = 0;
// Horizontal distance from bot center to vertical tracking wheel (in, + = right of center)
double vertical_tracker_dist_from_center = 0;
double horizontal_tracker_diameter = 2.75; // in
double vertical_tracker_diameter = 2;      // in

// ── Distance sensor position-reset offsets (only needed if used) ────────
// If a sensor is dead-center but mounted 6.5" forward, for example:
//   double front_sensor_offsetX = 0.0;
//   double front_sensor_offsetY = 6.5;
double front_sensor_offsetX = 0.0;
double front_sensor_offsetY = 0.0;

double left_sensor_offsetX = 0.0;
double left_sensor_offsetY = 0.0;

double right_sensor_offsetX = 0.0;
double right_sensor_offsetY = 0.0;

double back_sensor_offsetX = 0.0;
double back_sensor_offsetY = 0.0;

double distance_between_sensors = 0.0;

// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================

bool heading_correction = true; // Use heading correction when the bot is stationary

// true = more accuracy/smoothness, false = more speed
bool dir_change_start = true;   // Less accel/decel, expecting direction change at start
bool dir_change_end = true;     // Less accel/decel, expecting direction change at end

double min_output = 5; // Minimum output voltage while chaining movements

// Maximum allowed change in voltage output per 10 msec during movement
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;

// Prevents excess slipping during boomerang movements.
// Decrease for less drift/inconsistency, increase for more speed.
double chase_power = 2.5;

// ============================================================================
// DO NOT CHANGE ANYTHING BELOW
// ============================================================================

bool RemoteControlCodeEnabled = true;

// ── DRIVE SCHEME ─────────────────────────────────────────────────────────
//   TANK         — Axis3 left side, Axis2 right side
//   ARCADE       — Axis3 forward, Axis4 turn (both on left stick)
//   SPLIT_ARCADE — Axis3 forward, Axis1 turn (left stick fwd, right stick turn)
DriveScheme drive_scheme = TANK;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}
