#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include "motor-control.h"
#include "robot-config.h"

// ============================================================================
// INTERNAL STATE (DO NOT CHANGE)
// ============================================================================
bool is_turning = false;
double prev_left_output = 0, prev_right_output = 0;
double x_pos = 0, y_pos = 0;
double correct_angle = 0;
// Field config for distance resets, DO NOT CHANGE
double field_half_size = 70.25;  // Half field size in inches

// ============================================================================
// CHASSIS CONTROL FUNCTIONS
// ============================================================================

/*
 * Sets the voltage output for the left and right chassis motors.
 * - left_power: Voltage for the left side (in volts).
 * - right_power: Voltage for the right side (in volts).
 */
void driveChassis(double left_power, double right_power) {
  // Spin left and right chassis motors with specified voltages
  left_chassis.spin(fwd, left_power, voltageUnits::volt);
  right_chassis.spin(fwd, right_power, voltageUnits::volt);
}

/*
 * Stops both chassis motors with the specified brake type.
 * - type: Brake mode (coast, brake, or hold).
 */
void stopChassis(brakeType type) {
  // Stop left and right chassis motors using the given brake type
  left_chassis.stop(type);
  right_chassis.stop(type);
}

/*
 * Resets the rotation position of both chassis motors to zero.
 */
void resetChassis() {
  // Set both chassis motor encoders to zero
  left_chassis.setPosition(0, degrees);
  right_chassis.setPosition(0, degrees);
}

/*
 * Returns the current rotation of the left chassis motor in degrees.
 */
double getLeftRotationDegree() {
  // Get left chassis motor position in degrees
  return left_chassis.position(degrees);
}

/*
 * Returns the current rotation of the right chassis motor in degrees.
 */
double getRightRotationDegree() {
  // Get right chassis motor position in degrees
  return right_chassis.position(degrees);
}

/*
 * Normalizes an angle to be within +/-180 degrees of the current heading.
 * - angle: The target angle to normalize.
 */
double normalizeTarget(double angle) {
  // Adjust angle to be within +/-180 degrees of the inertial sensor's rotation
  if (angle - getInertialHeading() > 180) {
    while (angle - getInertialHeading() > 180) angle -= 360;
  } else if (angle - getInertialHeading() < -180) {
    while (angle - getInertialHeading() < -180) angle += 360;
  }
  return angle;
}

/*
 * Returns the current inertial sensor heading in degrees.
 * - normalize: If true, normalizes the heading (not used in this implementation).
 */
double getInertialHeading() {
  // Get inertial sensor rotation in degrees
  return inertial_sensor.rotation(degrees);
}

// ============================================================================
// OUTPUT SCALING HELPER FUNCTIONS
// ============================================================================

/*
 * Ensures output values are at least the specified minimum for both sides.
 * - left_output: Reference to left output voltage.
 * - right_output: Reference to right output voltage.
 * - min_output: Minimum allowed output voltage.
 */
void scaleToMin(double& left_output, double& right_output, double min_output) {
  // Scale outputs to ensure minimum voltage is met for both sides
  if (fabs(left_output) <= fabs(right_output) && left_output < min_output && left_output > 0) {
    right_output = right_output / left_output * min_output;
    left_output = min_output;
  } else if (fabs(right_output) < fabs(left_output) && right_output < min_output && right_output > 0) {
    left_output = left_output / right_output * min_output;
    right_output = min_output;
  } else if (fabs(left_output) <= fabs(right_output) && left_output > -min_output && left_output < 0) {
    right_output = right_output / left_output * -min_output;
    left_output = -min_output;
  } else if (fabs(right_output) < fabs(left_output) && right_output > -min_output && right_output < 0) {
    left_output = left_output / right_output * -min_output;
    right_output = -min_output;
  }
}

/*
 * Ensures output values do not exceed the specified maximum for both sides.
 * - left_output: Reference to left output voltage.
 * - right_output: Reference to right output voltage.
 * - max_output: Maximum allowed output voltage.
 */
void scaleToMax(double& left_output, double& right_output, double max_output) {
  // Scale outputs to ensure maximum voltage is not exceeded for both sides
  if (fabs(left_output) >= fabs(right_output) && left_output > max_output) {
    right_output = right_output / left_output * max_output;
    left_output = max_output;
  } else if (fabs(right_output) > fabs(left_output) && right_output > max_output) {
    left_output = left_output / right_output * max_output;
    right_output = max_output;
  } else if (fabs(left_output) > fabs(right_output) && left_output < -max_output) {
    right_output = right_output / left_output * -max_output;
    left_output = -max_output;
  } else if (fabs(right_output) > fabs(left_output) && right_output < -max_output) {
    left_output = left_output / right_output * -max_output;
    right_output = -max_output;
  }
}

// ============================================================================
// MAIN DRIVE AND TURN FUNCTIONS
// ============================================================================

/*
 * Turns the robot to a specified angle using PID control.
 * - turn_angle: Target angle to turn to (in degrees).
 * - time_limit_msec: Maximum time allowed for the turn (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void turnToAngle(double turn_angle, double time_limit_msec, bool exit, double max_output) {
  // Prepare for turn
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 1;
  PID pid = PID(turn_kp, turn_ki, turn_kd);

  // Normalize and set PID target
  turn_angle = normalizeTarget(turn_angle);
  pid.setTarget(turn_angle);
  pid.setIntegralMax(0);  
  pid.setIntegralRange(3);
  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);

  // Draw baseline for visualization
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // PID loop for turning
  double start_time = Brain.timer(msec);
  double output;
  double current_heading = getInertialHeading();
  double previous_heading = 0;
  int index = 1;
  if(exit == false && correct_angle < turn_angle) {
    // Turn right without stopping at end
    while (getInertialHeading() < turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading); // PID update for heading
      // Draw heading trace
      Brain.Screen.drawLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(output, -output);
      wait(10, msec);
    }
  } else if(exit == false && correct_angle > turn_angle) {
    // Turn left without stopping at end
    while (getInertialHeading() > turn_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      Brain.Screen.drawLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(-output, output);
      wait(10, msec);
    }
  } else {
    // Standard PID turn
    while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);
      Brain.Screen.drawLine(index * 3, fabs(previous_heading) * draw_amplifier, (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;
      driveChassis(output, -output);
      wait(10, msec);
    }
  }
  if(exit) {
    stopChassis(vex::hold);
  }
  correct_angle = turn_angle;
  is_turning = false;
}

/*
 * Drives the robot a specified distance (in inches) using PID control.
 * - distance_in: Target distance to drive (positive or negative).
 * - time_limit_msec: Maximum time allowed for the move (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void driveTo(double distance_in, double time_limit_msec, bool exit, double max_output) {
  // Store initial encoder values
  double start_left = getLeftRotationDegree(), start_right = getRightRotationDegree();
  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 0.5;
  int drive_direction = distance_in > 0 ? 1 : -1;
  double max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // Adjust slew rates and min speed for chaining
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = drive_direction > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  distance_in = distance_in * drive_direction;
  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // Configure PID controllers
  pid_distance.setTarget(distance_in);
  pid_distance.setIntegralMax(3);  
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(correct_angle));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_distance = 0, current_angle = 0;

  // Main PID loop for driving straight
  while (((!pid_distance.targetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec && exit) || (exit == false && current_distance < distance_in && Brain.timer(msec) - start_time <= time_limit_msec)) {
    // Calculate current distance and heading
    current_distance = (fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in) + fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in)) / 2;
    current_angle = getInertialHeading();
    left_output = pid_distance.update(current_distance) * drive_direction;
    right_output = left_output;
    correction_output = pid_heading.update(current_angle);

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }
    if(!exit) {
      left_output = 24 * drive_direction;
      right_output = 24 * drive_direction;
    }

    left_output += correction_output;
    right_output -= correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output);
    wait(10, msec);
  }
  if(exit) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::hold);
  }
  is_turning = false;
}

/*
 * CurveCircle
 * Drives the robot in a circular arc with a specified radius and angle.
 * - result_angle_deg: Target ending angle (in degrees) for the arc.
 * - center_radius: Radius of the circle's center (positive for curve to the right, negative for curve to the left).
 * - time_limit_msec: Maximum time allowed for the curve (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit, double max_output) {
  // Store initial encoder values for both sides
  double start_right = getRightRotationDegree(), start_left = getLeftRotationDegree();
  double in_arc, out_arc;
  double real_angle = 0, current_angle = 0;
  double ratio, result_angle;

  // Normalize the target angle to be within +/-180 degrees of the current heading
  result_angle_deg = normalizeTarget(result_angle_deg);
  result_angle = (result_angle_deg - correct_angle) * 3.14159265359 / 180;

  // Calculate arc lengths for inner and outer wheels
  in_arc = fabs((fabs(center_radius) - (distance_between_wheels / 2)) * result_angle);
  out_arc = fabs((fabs(center_radius) + (distance_between_wheels / 2)) * result_angle);

  // FIX: Prevent division by zero when out_arc is near zero (straight line case)
  if (fabs(out_arc) < 1e-6) {
    ratio = 1.0;  // For straight line, inner and outer arcs are equal
  } else {
    ratio = in_arc / out_arc;
  }

  stopChassis(vex::brakeType::coast);
  is_turning = true;
  double threshold = 0.5;

  // Determine curve and drive direction
  int curve_direction = center_radius > 0 ? 1 : -1;
  int drive_direction = 0;
  if ((curve_direction == 1 && (result_angle_deg - correct_angle) > 0) || (curve_direction == -1 && (result_angle_deg - correct_angle) < 0)) {
    drive_direction = 1;
  } else {
    drive_direction = -1;
  }

  // Slew rate and minimum speed logic for chaining
  double max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = drive_direction > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = drive_direction > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = drive_direction > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  // Initialize PID controllers for arc distance and heading correction
  PID pid_out = PID(distance_kp, distance_ki, distance_kd);
  PID pid_turn = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid_out.setTarget(out_arc);
  pid_out.setIntegralMax(0);
  pid_out.setIntegralRange(5);
  pid_out.setSmallBigErrorTolerance(0.3, 0.9);
  pid_out.setSmallBigErrorDuration(50, 250);
  pid_out.setDerivativeTolerance(threshold * 4.5);

  pid_turn.setTarget(0);
  pid_turn.setIntegralMax(0);
  pid_turn.setIntegralRange(1);
  pid_turn.setSmallBigErrorTolerance(0, 0);
  pid_turn.setSmallBigErrorDuration(0, 0);
  pid_turn.setDerivativeTolerance(0);
  pid_turn.setArrive(false);

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0;
  double current_right = 0, current_left = 0;

  // Main control loop for each curve/exit configuration
  if (curve_direction == -1 && exit == true) {
    // Left curve, stop at end
    while (!pid_out.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
      // Calculate the real angle along the arc
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      // Enforce minimum output if chaining
      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      // Apply heading correction
      left_output += correction_output;
      right_output -= correction_output;

      // Enforce maximum output
      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == 1 && exit == true) {
    // Right curve, stop at end
    while (!pid_out.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else if (curve_direction == -1 && exit == false) {
    // Left curve, chaining (do not stop at end)
    while (current_right < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_right = fabs(((getRightRotationDegree() - start_right) / 360.0) * wheel_distance_in);
      real_angle = current_right/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      right_output = pid_out.update(current_right) * drive_direction;
      left_output = right_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  } else {
    // Right curve, chaining (do not stop at end)
    while (current_left < out_arc && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_angle = getInertialHeading();
      current_left = fabs(((getLeftRotationDegree() - start_left) / 360.0) * wheel_distance_in);
      real_angle = current_left/out_arc * (result_angle_deg - correct_angle) + correct_angle;
      pid_turn.setTarget(normalizeTarget(real_angle));
      left_output = pid_out.update(current_left) * drive_direction;
      right_output = left_output * ratio;
      correction_output = pid_turn.update(current_angle);

      if(min_speed) {
        scaleToMin(left_output, right_output, min_output);
      }

      left_output += correction_output;
      right_output -= correction_output;

      scaleToMax(left_output, right_output, max_output);

      driveChassis(left_output, right_output);
      wait(10, msec);
    }
  }
  // Stop the chassis if required
  if(exit == true) {
    stopChassis(vex::brakeType::hold);
  }
  // Update the global heading
  correct_angle = result_angle_deg;
  is_turning = false;
}

/*
 * Swing
 * Performs a swing turn, rotating the robot around a point while driving forward or backward.
 * - swing_angle: Target angle to swing to (in degrees).
 * - drive_direction: Direction to drive (1 for forward, -1 for backward).
 * - time_limit_msec: Maximum time allowed for the swing (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 */
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit, double max_output) {
  stopChassis(vex::brakeType::coast); // Stop chassis before starting swing
  is_turning = true;                  // Set turning state
  double threshold = 1;
  PID pid = PID(turn_kp, turn_ki, turn_kd); // Initialize PID for turning

  swing_angle = normalizeTarget(swing_angle); // Normalize target angle
  pid.setTarget(swing_angle);                 // Set PID target
  pid.setIntegralMax(0);  
  pid.setIntegralRange(5);

  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(50, 250);
  pid.setDerivativeTolerance(threshold * 4.5);

  // Draw the baseline for visualization
  double draw_amplifier = 230 / fabs(swing_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(swing_angle) * draw_amplifier, 600, fabs(swing_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // Start the PID loop
  double start_time = Brain.timer(msec);
  double output;
  double current_heading = correct_angle;
  double previous_heading = 0;
  int index = 1;
  int choice = 1;

  // Determine which side to swing and direction
  if(swing_angle - correct_angle < 0 && drive_direction == 1) {
    choice = 1;
  } else if(swing_angle - correct_angle > 0 && drive_direction == 1) {
    choice = 2;
  } else if(swing_angle - correct_angle < 0 && drive_direction == -1) {
    choice = 3;
  } else {
    choice = 4;
  }

  // Swing logic for each case, chaining (exit == false)
  if(choice == 1 && exit == false) {
    // Swing left, forward
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.stop(hold); // Hold left, swing right
      right_chassis.spin(fwd, output * drive_direction, volt);
      wait(10, msec);
    }
  } else if(choice == 2 && exit == false) {
    // Swing right, forward
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold); // Hold right, swing left
      wait(10, msec);
    }
  } else if(choice == 3 && exit == false) {
    // Swing left, backward
    while (current_heading > swing_angle && Brain.timer(msec) - start_time <= time_limit_msec) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold);
      wait(10, msec);
    }
  } else {
    // Swing right, backward
    while (current_heading < swing_angle && Brain.timer(msec) - start_time <= time_limit_msec && exit == false) {
      current_heading = getInertialHeading();
      output = pid.update(current_heading);

      // Draw heading trace
      Brain.Screen.drawLine(
          index * 3, fabs(previous_heading) * draw_amplifier, 
          (index + 1) * 3, fabs(current_heading * draw_amplifier));
      index++;
      previous_heading = current_heading;

      // Clamp output
      if(output < min_output) output = min_output;
      if(output > max_output) output = max_output;
      else if(output < -max_output) output = -max_output;

      left_chassis.stop(hold);
      right_chassis.spin(fwd, output * drive_direction, volt);
      wait(10, msec);
    }
  }

  // PID loop for exit == true (stop at end)
  while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec && exit == true) {
    current_heading = getInertialHeading();
    output = pid.update(current_heading);

    // Draw heading trace
    Brain.Screen.drawLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;

    // Clamp output
    if(output > max_output) output = max_output;
    else if(output < -max_output) output = -max_output;

    // Apply output to correct side based on swing direction
    switch(choice) {
    case 1:
      left_chassis.stop(hold);
      right_chassis.spin(fwd, -output * drive_direction, volt);
      break;
    case 2:
      left_chassis.spin(fwd, output * drive_direction, volt);
      right_chassis.stop(hold);
      break;
    case 3:
      left_chassis.spin(fwd, -output * drive_direction, volt);
      right_chassis.stop(hold);
      break;
    case 4:
      left_chassis.stop(hold);
      right_chassis.spin(fwd, output * drive_direction, volt);
      break;
    }
    wait(10, msec);
  }
  if(exit == true) {
    stopChassis(vex::hold); // Stop chassis at end if required
  }
  correct_angle = swing_angle; // Update global heading
  is_turning = false;          // Reset turning state
}

/*
 * heading_correction
 * Continuously adjusts the robot's heading to maintain a straight course.
 * Uses a PID controller to minimize the error between the current heading and the target heading.
 */
void correctHeading() {
  double output = 0;
  PID pid = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid.setTarget(correct_angle); // Set PID target to current heading
  pid.setIntegralRange(fabs(correct_angle) / 2.5);

  pid.setSmallBigErrorTolerance(0, 0);
  pid.setSmallBigErrorDuration(0, 0);
  pid.setDerivativeTolerance(0);
  pid.setArrive(false);

  // Continuously correct heading while enabled
  while(heading_correction) {
    pid.setTarget(correct_angle);
    if(is_turning == false) {
      output = pid.update(getInertialHeading());
      driveChassis(output, -output); // Apply correction to chassis
    }
    wait(10, msec);
  }
}

/*
 * trackOdom
 * Replaces trackNoOdomWheel / trackXOdomWheel / trackYOdomWheel /
 * trackXYOdomWheel with a single function, structured to match how LemLib's
 * TrackingWheelOdometry::update() actually works (calhighrobotics/push_back_x
 * uses LemLib for odometry -- see their firmware/LemLib.a and
 * include/lemlib/chassis/odom.hpp -- and LemLib's own tracking
 * implementation is itself an application of the same 5225A/Pilons paper
 * this template's math is already based on: http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf).
 *
 * The core formula is unchanged from your original functions -- it was
 * already correct and matches LemLib's:
 *
 *   local_axis_delta = 2*sin(dtheta/2) * (raw_delta/dtheta + offset)   [dtheta != 0]
 *   local_axis_delta = raw_delta                                       [dtheta == 0]
 *
 * applied independently per axis (horizontal tracker -> local X, vertical
 * tracker/drive encoders -> local Y), then rotated into the global frame by
 * (previous_heading + dtheta/2) -- same "rotate by the average heading
 * during this timestep" trick your code already used.
 *
 * What changes vs. the four separate functions:
 *
 * 1. ONE function instead of four near-duplicates, driven by the existing
 *    using_horizontal_tracker / using_vertical_tracker flags in
 *    robot-config.cpp, so switching tracker configurations doesn't mean
 *    switching which function you call.
 *
 * 2. FIXES THE trackYOdomWheel BUG: that function never computed a local_x
 *    term at all -- it silently assumed zero sideways motion always, so any
 *    lateral slip (defense contact, turning scrub) was invisible and
 *    permanently corrupted x_pos with no correction. Here, when there's no
 *    horizontal tracker, local_x is explicitly set to 0 with a comment
 *    saying why, instead of just never being computed -- same real-world
 *    limitation (you cannot sense an axis you have no sensor for), but now
 *    it's a documented, deliberate choice instead of an accidental gap, and
 *    it goes through the exact same rotation math as every other case so
 *    there's no separate/inconsistent polar-angle branch like the original
 *    trackYOdomWheel had.
 *
 * 3. Falls back to drivetrain wheel encoders for the Y axis if there's no
 *    vertical tracker (matching your original trackXOdomWheel behavior --
 *    this is better than LemLib's own default of just returning 0 for a
 *    missing axis, at the cost of being slip-prone under load).
 */
void trackOdom() {
  resetChassis();
  double prev_heading_rad = 0;
  double prev_horizontal_pos_deg = 0, prev_vertical_pos_deg = 0;
  double prev_left_deg = 0, prev_right_deg = 0;
 
  while (true) {
    double heading_rad = degToRad(getInertialHeading());
    double delta_heading_rad = heading_rad - prev_heading_rad;
 
    // --- Local X (sideways) delta ---
    double delta_local_x_in;
    if (using_horizontal_tracker) {
      double horizontal_pos_deg = horizontal_tracker.position(degrees);
      double delta_horizontal_in = (horizontal_pos_deg - prev_horizontal_pos_deg) * horizontal_tracker_diameter * M_PI / 360.0;
      if (fabs(delta_heading_rad) < 1e-6) {
        delta_local_x_in = delta_horizontal_in;
      } else {
        double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
        delta_local_x_in = sin_multiplier * ((delta_horizontal_in / delta_heading_rad) + horizontal_tracker_dist_from_center);
      }
      prev_horizontal_pos_deg = horizontal_pos_deg;
    } else {
      // No sensor for this axis -- cannot detect sideways slip. Documented
      // limitation, not a silent gap: any lateral drift will not be caught.
      delta_local_x_in = 0;
    }
 
    // --- Local Y (forward) delta ---
    double delta_local_y_in;
    if (using_vertical_tracker) {
      double vertical_pos_deg = vertical_tracker.position(degrees);
      double delta_vertical_in = (vertical_pos_deg - prev_vertical_pos_deg) * vertical_tracker_diameter * M_PI / 360.0;
      if (fabs(delta_heading_rad) < 1e-6) {
        delta_local_y_in = delta_vertical_in;
      } else {
        double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
        delta_local_y_in = sin_multiplier * ((delta_vertical_in / delta_heading_rad) + vertical_tracker_dist_from_center);
      }
      prev_vertical_pos_deg = vertical_pos_deg;
    } else {
      // Fall back to drivetrain encoders. Better than assuming 0, but these
      // wheels are traction-limited and will slip under contact/pushing.
      double left_deg = getLeftRotationDegree();
      double right_deg = getRightRotationDegree();
      double delta_left_in = (left_deg - prev_left_deg) * wheel_distance_in / 360.0;
      double delta_right_in = (right_deg - prev_right_deg) * wheel_distance_in / 360.0;
      if (fabs(delta_heading_rad) < 1e-6) {
        delta_local_y_in = (delta_left_in + delta_right_in) / 2.0;
      } else {
        double sin_multiplier = 2.0 * sin(delta_heading_rad / 2.0);
        double delta_local_y_left_in = sin_multiplier * (delta_left_in / delta_heading_rad + distance_between_wheels / 2.0);
        double delta_local_y_right_in = sin_multiplier * (delta_right_in / delta_heading_rad + distance_between_wheels / 2.0);
        delta_local_y_in = (delta_local_y_left_in + delta_local_y_right_in) / 2.0;
      }
      prev_left_deg = left_deg;
      prev_right_deg = right_deg;
    }
 
    // --- Rotate local (x, y) delta into the global frame ---
    // Same approach as before: treat the heading during this timestep as
    // the average of the heading at the start and end of the step.
    double local_polar_angle_rad;
    if (fabs(delta_local_x_in) < 1e-6 && fabs(delta_local_y_in) < 1e-6) {
      local_polar_angle_rad = 0;
    } else {
      local_polar_angle_rad = atan2(delta_local_y_in, delta_local_x_in);
    }
    double polar_radius_in = sqrt(pow(delta_local_x_in, 2) + pow(delta_local_y_in, 2));
    double polar_angle_rad = local_polar_angle_rad - heading_rad - (delta_heading_rad / 2);
 
    x_pos += polar_radius_in * cos(polar_angle_rad);
    y_pos += polar_radius_in * sin(polar_angle_rad);
 
    prev_heading_rad = heading_rad;
 
    wait(10, msec);
  }
}

/*
 * turnToPoint
 * Turns the robot to face a specific point in the field.
 * - x, y: Coordinates of the target point.
 * - direction: Direction to face the point (1 for forward, -1 for backward).
 * - time_limit_msec: Maximum time allowed for the turn (in milliseconds).
 */
void turnToPoint(double x, double y, int direction, double time_limit_msec) {
  stopChassis(vex::brakeType::coast); // Stop chassis before turning
  is_turning = true;                  // Set turning state
  double threshold = 1, add = 0;
  if(direction == -1) {
    add = 180; // Add 180 degrees if turning to face backward
  }
  // Calculate target angle using atan2 and normalize
  double turn_angle = normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add);
  PID pid = PID(turn_kp, turn_ki, turn_kd);

  pid.setTarget(turn_angle); // Set PID target
  pid.setIntegralMax(0);  
  pid.setIntegralRange(3);

  pid.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid.setSmallBigErrorDuration(100, 500);
  pid.setDerivativeTolerance(threshold * 4.5);

  // Draw the baseline for visualization
  double draw_amplifier = 230 / fabs(turn_angle);
  Brain.Screen.clearScreen(black);
  Brain.Screen.setPenColor(green);
  Brain.Screen.drawLine(0, fabs(turn_angle) * draw_amplifier, 
                        600, fabs(turn_angle) * draw_amplifier);
  Brain.Screen.setPenColor(red);

  // Start the PID loop
  double start_time = Brain.timer(msec);
  double output;
  double current_heading;
  double previous_heading = 0;
  int index = 1;
  while (!pid.targetArrived() && Brain.timer(msec) - start_time <= time_limit_msec) {
    // Continuously update target as robot moves
    pid.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
    current_heading = getInertialHeading();
    output = pid.update(current_heading);

    // Draw heading trace
    Brain.Screen.drawLine(
        index * 3, fabs(previous_heading) * draw_amplifier, 
        (index + 1) * 3, fabs(current_heading * draw_amplifier));
    index++;
    previous_heading = current_heading;

    driveChassis(output, -output); // Apply output to chassis
    wait(10, msec);
  }  
  stopChassis(vex::hold); // Stop at end
  correct_angle = getInertialHeading(); // Update global heading
  is_turning = false;                   // Reset turning state
}

/*
 * moveToPoint
 * Moves the robot to a specific point in the field, adjusting heading as needed.
 * - x, y: Coordinates of the target point.
 * - dir: Direction to move in (1 for forward, -1 for backward).
 * - time_limit_msec: Maximum time allowed for the move (in milliseconds).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 * - overturn: If true, allows overturning for sharp turns.
 */
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn) {
  stopChassis(vex::brakeType::coast); // Stop chassis before moving
  is_turning = true;                  // Set turning state
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // Adjust slew rates and min speed for chaining
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  // Set PID targets for distance and heading
  pid_distance.setTarget(hypot(x - x_pos, y - y_pos));
  pid_distance.setIntegralMax(0);  
  pid_distance.setIntegralRange(3);
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);
  
  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  // Reset the chassis
  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0, prev_left_output = 0, prev_right_output = 0;
  double exittolerance = 1;
  bool perpendicular_line = false, prev_perpendicular_line = true;

  double current_angle = 0, overturn_value = 0;
  bool ch = true;

  // Main PID loop for moving to point
  while (Brain.timer(msec) - start_time <= time_limit_msec) {
    // Continuously update targets as robot moves
    pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
    pid_distance.setTarget(hypot(x - x_pos, y - y_pos));
    current_angle = getInertialHeading();
    // Calculate drive output based on heading and distance
    left_output = pid_distance.update(0) * cos(degToRad(atan2(x - x_pos, y - y_pos) * 180 / M_PI + add - current_angle)) * dir;
    right_output = left_output;
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((y_pos - y) * -cos(degToRad(normalizeTarget(current_angle + add))) <= (x_pos - x) * sin(degToRad(normalizeTarget(current_angle + add))) + exittolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Only apply heading correction if far from target
    if(hypot(x - x_pos, y - y_pos) > 8 && ch == true) {
      correction_output = pid_heading.update(current_angle);
    } else {
      correction_output = 0;
      ch = false;
    }

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }

    // Overturn logic for sharp turns
    overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
    if(overturn_value > 0 && overturn) {
      if(left_output > 0) {
        left_output -= overturn_value;
      }
      else {
        left_output += overturn_value;
      }
    }
    right_output = left_output;
    left_output = left_output + correction_output;
    right_output = right_output - correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output); // Apply output to chassis
    wait(10, msec);
  }
  if(exit == true) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::hold); // Stop at end if required
  }
  correct_angle = getInertialHeading(); // Update global heading
  is_turning = false;                   // Reset turning state
}

/*
 * moveToPointField
 * A local-frame version of moveToPoint, adapted from a Ramsete-style
 * controller (rotation-matrix error decomposition + explicit settle phase)
 * to this template's tools: PID class, x_pos/y_pos, getInertialHeading(),
 * driveChassis(). No PROS -- uses Brain.timer(msec)/wait(msec) like the
 * rest of the file.
 *
 * The key structural difference from the original moveToPoint:
 *
 * 1. LOCAL-FRAME ERROR instead of atan2+normalizeTarget angle math. The
 *    global error (target - current position) is rotated into the robot's
 *    own heading frame using a standard 2D rotation:
 *        local_fwd = cos(theta)*dy + sin(theta)*dx
 *        local_lat = -sin(theta)*dy + cos(theta)*dx
 *    This sidesteps angle-wraparound bugs entirely (no normalizeTarget
 *    needed) because everything is just forward/lateral distance in the
 *    robot's own frame, not an absolute heading target.
 *
 * 2. EXPLICIT SETTLE PHASE. Once the robot is within settle_dist of the
 *    target, angular correction is dropped to 0 and drive_error becomes the
 *    signed forward distance to the target (local_fwd) instead of the raw
 *    magnitude -- so the last few inches are a straight-line correction
 *    along the robot's current heading, not a re-aimed turn-and-drive.
 *    This is what fixes the "brakes so suddenly" problem from before: the
 *    settle phase drives drive_error, and therefore drive_output, smoothly
 *    to 0 as the robot arrives, instead of exiting mid-ramp with leftover
 *    voltage that then gets killed by stopChassis(hold) in one step.
 *
 * 3. SUSTAINED EXIT CONDITION. Exits only after both local_fwd and
 *    local_lat stay within tolerance for exit_hold_msec continuously (like
 *    lemlib's ExitCondition), not a one-shot geometry check that can
 *    flicker near the target.
 *
 * - x, y: target point.
 * - dir: 1 = forward, -1 = backward.
 * - time_limit_msec: hard timeout.
 * - exit: if true, hold-brakes at the end; if false, leaves velocity live
 *   for chaining (same convention as the rest of this file).
 * - max_output: voltage cap.
 * - overturn: if true, allows the angular term to eat into the drive
 *   budget on sharp turns instead of being capped independently.
 */
void moveToPointField(double x, double y, int dir, double time_limit_msec, bool exit, double max_output, bool overturn) {
  stopChassis(vex::brakeType::coast);
  is_turning = true;
 
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }
 
  PID pid_lateral = PID(distance_kp, distance_ki, distance_kd);
  PID pid_angular = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);
 
  double settle_dist = 4.0;      // inches: switch from turn-and-drive to straight-line settle
  double taper_dist = 12.0;      // inches: start tapering angular authority here, before settle_dist
  double exit_tolerance = 0.75;  // inches, both axes
  double exit_hold_msec = 150;   // must stay within tolerance this long to exit
  double exit_hold_start = -1;
 
  bool settling = false;
  double start_time = Brain.timer(msec);
  double prev_left_output = 0, prev_right_output = 0;
 
  while (Brain.timer(msec) - start_time <= time_limit_msec) {
    double heading_deg = getInertialHeading();
    double theta = degToRad(heading_deg);
 
    double dx = x - x_pos;
    double dy = y - y_pos;
    if(dir < 0) { dx = -dx; dy = -dy; } // treat "backward" as driving into the flipped point
 
    // Rotate global error into the robot's local frame.
    double local_fwd = cos(theta) * dy + sin(theta) * dx;
    double local_lat = -sin(theta) * dy + cos(theta) * dx;
 
    double d = hypot(dx, dy);
    if(d < settle_dist) settling = true;
 
    double angular_error_deg;
    double cosine_scale;
    double drive_error;
 
    if(settling) {
      angular_error_deg = 0;
      cosine_scale = 1.0;
      drive_error = local_fwd; // signed straight-line distance remaining
    } else {
      double angular_error_rad = atan2(local_lat, local_fwd);
      angular_error_deg = radToDeg(angular_error_rad);
      cosine_scale = cos(angular_error_rad);
      drive_error = d * (cosine_scale >= 0 ? 1.0 : -1.0);
 
      // Taper angular authority down as d approaches settle_dist, instead of
      // trusting atan2 right up until the hard switch into settling. Near
      // settle_dist, a small lateral offset with a small local_fwd makes
      // atan2 swing toward +-90 deg on noise alone, which otherwise causes
      // the robot to spin in place chasing a target angle that itself
      // slides around as the robot turns (rotating theta recomputes
      // local_fwd/local_lat every loop).
      double taper_weight = (d - settle_dist) / (taper_dist - settle_dist);
      if(taper_weight > 1) taper_weight = 1;
      if(taper_weight < 0) taper_weight = 0;
      angular_error_deg *= taper_weight;
    }
 
    // Sustained exit condition -- both axes in tolerance for exit_hold_msec.
    if(fabs(local_fwd) < exit_tolerance && fabs(local_lat) < exit_tolerance) {
      if(exit_hold_start < 0) exit_hold_start = Brain.timer(msec);
      if(Brain.timer(msec) - exit_hold_start >= exit_hold_msec) break;
    } else {
      exit_hold_start = -1;
    }
 
    pid_lateral.setTarget(drive_error);
    pid_angular.setTarget(angular_error_deg);
    double drive_output = pid_lateral.update(0);
    double angular_output = settling ? 0 : pid_angular.update(0);
 
    drive_output = fmax(fmin(drive_output, max_output), -max_output);
    drive_output *= fabs(cosine_scale); // ease off drive while heading is far off, ramp back in as it aligns
 
    if(dir < 0) drive_output = -drive_output; // undo the earlier flip for actual motor sign
 
    double left_output = drive_output + angular_output;
    double right_output = drive_output - angular_output;
 
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }
 
    if(overturn) {
      double overturn_value = fabs(drive_output) + fabs(angular_output) - max_output;
      if(overturn_value > 0) {
        if(drive_output > 0) drive_output -= overturn_value;
        else drive_output += overturn_value;
        left_output = drive_output + angular_output;
        right_output = drive_output - angular_output;
      }
    }
 
    scaleToMax(left_output, right_output, max_output);
 
    if(prev_left_output - left_output > max_slew_rev) left_output = prev_left_output - max_slew_rev;
    if(prev_right_output - right_output > max_slew_rev) right_output = prev_right_output - max_slew_rev;
    if(left_output - prev_left_output > max_slew_fwd) left_output = prev_left_output + max_slew_fwd;
    if(right_output - prev_right_output > max_slew_fwd) right_output = prev_right_output + max_slew_fwd;
    prev_left_output = left_output;
    prev_right_output = right_output;
 
    driveChassis(left_output, right_output);
    wait(10, msec);
  }
 
  if(exit) {
    stopChassis(vex::hold);
  }
  correct_angle = getInertialHeading();
  is_turning = false;
}

/*
 * boomerang
 * Drives the robot in a boomerang-shaped path to a target point.
 * - x, y: Coordinates of the target point.
 * - a: Final angle of the robot to target (in degrees).
 * - dlead: Distance to lead the target by (in inches, set higher for curvier path, don't set above 0.6).
 * - time_limit_msec: Maximum time allowed for the maneuver (in milliseconds).
 * - dir: Direction to move in (1 for forward, -1 for backward).
 * - exit: If true, stops the robot at the end; if false, allows chaining.
 * - max_output: Maximum voltage output to motors.
 * - overturn: If true, allows overturning for sharp turns.
 */
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit, double max_output, bool overturn) {
  stopChassis(vex::brakeType::coast); // Stop chassis before moving
  is_turning = true;                  // Set turning state
  double threshold = 0.5;
  int add = dir > 0 ? 0 : 180;
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    // Adjust slew rates and min speed for chaining
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }

  PID pid_distance = PID(distance_kp, distance_ki, distance_kd);
  PID pid_heading = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);

  pid_distance.setTarget(0); // Target is dynamically updated
  pid_distance.setIntegralMax(3);  
  pid_distance.setSmallBigErrorTolerance(threshold, threshold * 3);
  pid_distance.setSmallBigErrorDuration(50, 250);
  pid_distance.setDerivativeTolerance(5);

  pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos))));
  pid_heading.setIntegralMax(0);  
  pid_heading.setIntegralRange(1);
  pid_heading.setSmallBigErrorTolerance(0, 0);
  pid_heading.setSmallBigErrorDuration(0, 0);
  pid_heading.setDerivativeTolerance(0);
  pid_heading.setArrive(false);

  double start_time = Brain.timer(msec);
  double left_output = 0, right_output = 0, correction_output = 0, slip_speed = 0, overturn_value = 0;
  double exit_tolerance = 3;
  bool perpendicular_line = false, prev_perpendicular_line = true;
  double current_angle = 0, hypotenuse = 0, carrot_x = 0, carrot_y = 0;

  // Main PID loop for boomerang path
  while ((!pid_distance.targetArrived()) && Brain.timer(msec) - start_time <= time_limit_msec) {
    hypotenuse = hypot(x_pos - x, y_pos - y); // Distance to target
    // Calculate carrot point for path leading
    carrot_x = x - hypotenuse * sin(degToRad(a + add)) * dlead;
    carrot_y = y - hypotenuse * cos(degToRad(a + add)) * dlead;
    pid_distance.setTarget(hypot(carrot_x - x_pos, carrot_y - y_pos) * dir);
    current_angle = getInertialHeading();
    // Calculate drive output based on carrot point
    left_output = pid_distance.update(0) * cos(degToRad(atan2(carrot_x - x_pos, carrot_y - y_pos) * 180 / M_PI + add - current_angle));
    right_output = left_output;
    // Check if robot has crossed the perpendicular line to the target
    perpendicular_line = ((y_pos - y) * -cos(degToRad(normalizeTarget(a))) <= (x_pos - x) * sin(degToRad(normalizeTarget(a))) + exit_tolerance);
    if(perpendicular_line && !prev_perpendicular_line) {
      break;
    }
    prev_perpendicular_line = perpendicular_line;

    // Minimum Output Check
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }

    // Heading correction logic based on distance to carrot/target
    if(hypot(carrot_x - x_pos, carrot_y - y_pos) > 8) {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(carrot_x - x_pos, carrot_y - y_pos)) + add));
      correction_output = pid_heading.update(current_angle);
    } else if(hypot(x - x_pos, y - y_pos) > 6) {
      pid_heading.setTarget(normalizeTarget(radToDeg(atan2(x - x_pos, y - y_pos)) + add));
      correction_output = pid_heading.update(current_angle);
    } else {
      pid_heading.setTarget(normalizeTarget(a));
      correction_output = pid_heading.update(current_angle);
      if(exit && hypot(x - x_pos, y - y_pos) < 5) {
        break;
      }
    }

    // Limit slip speed for smoother curves
    slip_speed = sqrt(chase_power * getRadius(x_pos, y_pos, carrot_x, carrot_y, current_angle) * 9.8);
    if(left_output > slip_speed) {
      left_output = slip_speed;
    } else if(left_output < -slip_speed) {
      left_output = -slip_speed;
    }

    // Overturn logic for sharp turns
    overturn_value = fabs(left_output) + fabs(correction_output) - max_output;
    if(overturn_value > 0 && overturn) {
      if(left_output > 0) {
        left_output -= overturn_value;
      }
      else {
        left_output += overturn_value;
      }
    }
    right_output = left_output;
    left_output = left_output + correction_output;
    right_output = right_output - correction_output;

    // Max Output Check
    scaleToMax(left_output, right_output, max_output);

    // Max Acceleration/Deceleration Check
    if(prev_left_output - left_output > max_slew_rev) {
      left_output = prev_left_output - max_slew_rev;
    }
    if(prev_right_output - right_output > max_slew_rev) {
      right_output = prev_right_output - max_slew_rev;
    }
    if(left_output - prev_left_output > max_slew_fwd) {
      left_output = prev_left_output + max_slew_fwd;
    }
    if(right_output - prev_right_output > max_slew_fwd) {
      right_output = prev_right_output + max_slew_fwd;
    }
    prev_left_output = left_output;
    prev_right_output = right_output;
    driveChassis(left_output, right_output); // Apply output to chassis
    wait(10, msec);
  }
  if(exit) {
    prev_left_output = 0;
    prev_right_output = 0;
    stopChassis(vex::hold); // Stop at end if required
  }
  correct_angle = a;      // Update global heading
  is_turning = false;     // Reset turning state
}

/*
 * boomerangField
 * Local-frame, carrot-point version of boomerang, adapted from a
 * Ramsete-style moveToPose (same family as moveToPointField) to this
 * template's tools: PID class, x_pos/y_pos, getInertialHeading(),
 * driveChassis(). No PROS -- uses Brain.timer(msec)/wait(msec).
 *
 * Same local-frame rotation trick as moveToPointField (cos/sin theta
 * instead of atan2+normalizeTarget), plus two things your original
 * boomerang() didn't have:
 *
 * 1. CARROT POINT INSTEAD OF RAW dlead BLEND. Your original boomerang()
 *    blends the raw target point with a point projected backward along the
 *    final approach angle, scaled by dlead, and re-aims at that blended
 *    point every loop. Here, the carrot point is placed a fixed lookahead
 *    distance AHEAD along the approach line (scaled by how far out the
 *    robot still is: lookahead = max(d * lead, min_lookahead)), and the
 *    robot always steers at that carrot -- this is the same idea pure
 *    pursuit uses for path following, applied to a single boomerang curve.
 *    Because the carrot stays offset from the true target until very close,
 *    local_fwd relative to the carrot never collapses toward 0 the way it
 *    does when aiming directly at a close target -- which is what caused
 *    the "spins around the point" bug in the earlier moveToPointField
 *    version. No taper needed here for that reason.
 *
 * 2. BLENDED DRIVE ERROR. drive_error uses
 *    sqrt((local_error^2 + 2*d^2) / 3) instead of raw local_fwd distance to
 *    the carrot -- this blends "distance to carrot" with "distance to true
 *    target" so speed doesn't overshoot just because the carrot is still
 *    far out early in the curve.
 *
 * 3. EXPLICIT SETTLE PHASE with a FIXED final heading target (a, in
 *    degrees), not an atan2-derived one -- so unlike the angular term
 *    earlier in the curve, there's no divide-by-small-distance instability
 *    during settling; it's just "turn to face `a`" using ordinary heading
 *    error.
 *
 * - x, y: target point.
 * - dir: 1 = forward, -1 = backward.
 * - a: final heading in degrees the robot should be facing at the target.
 * - dlead: how far ahead of the robot to place the carrot, as a fraction of
 *   remaining distance (same tuning range as your original: don't set above
 *   ~0.6). Higher = curvier path.
 * - time_limit_msec: hard timeout.
 * - exit: if true, hold-brakes at the end; if false, leaves velocity live
 *   for chaining.
 * - max_output: voltage cap.
 * - overturn: if true, allows the angular term to eat into the drive
 *   budget on sharp turns instead of being capped independently.
 */
void boomerangField(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit, double max_output, bool overturn) {
  stopChassis(vex::brakeType::coast);
  is_turning = true;
 
  double max_slew_fwd = dir > 0 ? max_slew_accel_fwd : max_slew_decel_rev;
  double max_slew_rev = dir > 0 ? max_slew_decel_fwd : max_slew_accel_rev;
  bool min_speed = false;
  if(!exit) {
    if(!dir_change_start && dir_change_end) {
      max_slew_fwd = dir > 0 ? 24 : max_slew_decel_rev;
      max_slew_rev = dir > 0 ? max_slew_decel_fwd : 24;
    }
    if(dir_change_start && !dir_change_end) {
      max_slew_fwd = dir > 0 ? max_slew_accel_fwd : 24;
      max_slew_rev = dir > 0 ? 24 : max_slew_accel_rev;
      min_speed = true;
    }
    if(!dir_change_start && !dir_change_end) {
      max_slew_fwd = 24;
      max_slew_rev = 24;
      min_speed = true;
    }
  }
 
  PID pid_lateral = PID(distance_kp, distance_ki, distance_kd);
  PID pid_angular = PID(heading_correction_kp, heading_correction_ki, heading_correction_kd);
 
  double settle_dist = 4.0;
  double min_lookahead = 4.0;
  double exit_tolerance = 0.75;
  double exit_hold_msec = 150;
  double exit_hold_start = -1;
 
  double theta_final_rad = degToRad(a);
  double end_unit_x = sin(theta_final_rad);
  double end_unit_y = cos(theta_final_rad);
 
  bool settling = false;
  double start_time = Brain.timer(msec);
  double prev_left_output = 0, prev_right_output = 0;
 
  while (Brain.timer(msec) - start_time <= time_limit_msec) {
    double heading_deg = getInertialHeading();
    double theta = degToRad(heading_deg);
 
    double d = hypot(x - x_pos, y - y_pos);
    if(d < settle_dist) settling = true;
 
    double true_dx = x - x_pos, true_dy = y - y_pos;
    if(dir < 0) { true_dx = -true_dx; true_dy = -true_dy; }
    double true_local_fwd = cos(theta) * true_dy + sin(theta) * true_dx;
    double true_local_lat = -sin(theta) * true_dy + cos(theta) * true_dx;
 
    double angular_error_deg;
    double cosine_scale;
    double drive_error;
 
    if(settling) {

      angular_error_deg = normalizeAngle(a - heading_deg);
      cosine_scale = 1.0;
      drive_error = true_local_fwd;
    } else {

      double rx = x_pos - x, ry = y_pos - y;
      double along_track = rx * end_unit_x + ry * end_unit_y;
      double lookahead = fmax(d * dlead, min_lookahead);
      double ghost_along_track = along_track + lookahead * (dir > 0 ? 1.0 : -1.0);
      double carrot_x = x + ghost_along_track * end_unit_x;
      double carrot_y = y + ghost_along_track * end_unit_y;
 
      double dx = carrot_x - x_pos, dy = carrot_y - y_pos;
      if(dir < 0) { dx = -dx; dy = -dy; }
      double local_fwd = cos(theta) * dy + sin(theta) * dx;
      double local_lat = -sin(theta) * dy + cos(theta) * dx;
 
      double heading_error_rad = atan2(local_lat, local_fwd);
      angular_error_deg = radToDeg(heading_error_rad);
      cosine_scale = cos(heading_error_rad);
 
      double error_norm_sq = local_fwd * local_fwd + local_lat * local_lat;
      drive_error = sqrt((error_norm_sq + 2.0 * d * d) / 3.0) * (cosine_scale >= 0 ? 1.0 : -1.0);
    }
 
    if(fabs(true_local_fwd) < exit_tolerance && fabs(true_local_lat) < exit_tolerance) {
      if(exit_hold_start < 0) exit_hold_start = Brain.timer(msec);
      if(Brain.timer(msec) - exit_hold_start >= exit_hold_msec) break;
    } else {
      exit_hold_start = -1;
    }
 
    pid_lateral.setTarget(drive_error);
    pid_angular.setTarget(angular_error_deg);
    double drive_output = pid_lateral.update(0);
    double angular_output = pid_angular.update(0);
 
    drive_output = fmax(fmin(drive_output, max_output), -max_output);
    drive_output *= fabs(cosine_scale);
 
    if(dir < 0) drive_output = -drive_output;
 
    double left_output = drive_output + angular_output;
    double right_output = drive_output - angular_output;
 
    if(min_speed) {
      scaleToMin(left_output, right_output, min_output);
    }
 
    if(overturn) {
      double overturn_value = fabs(drive_output) + fabs(angular_output) - max_output;
      if(overturn_value > 0) {
        if(drive_output > 0) drive_output -= overturn_value;
        else drive_output += overturn_value;
        left_output = drive_output + angular_output;
        right_output = drive_output - angular_output;
      }
    }
 
    scaleToMax(left_output, right_output, max_output);
 
    if(prev_left_output - left_output > max_slew_rev) left_output = prev_left_output - max_slew_rev;
    if(prev_right_output - right_output > max_slew_rev) right_output = prev_right_output - max_slew_rev;
    if(left_output - prev_left_output > max_slew_fwd) left_output = prev_left_output + max_slew_fwd;
    if(right_output - prev_right_output > max_slew_fwd) right_output = prev_right_output + max_slew_fwd;
    prev_left_output = left_output;
    prev_right_output = right_output;
 
    driveChassis(left_output, right_output);
    wait(10, msec);
  }
 
  if(exit) {
    stopChassis(vex::hold);
  }
  correct_angle = getInertialHeading();
  is_turning = false;
}

// ============================================================================
// DISTANCE SENSOR POSITION RESET FUNCTIONS
// ============================================================================

/*
 * resetPositionWithSensor
 * Resets position using a distance sensor based on which wall the robot is facing.
 * Only resets X or Y position (not both) based on the wall being measured.
 * 
 * - sensor: Distance sensor to use
 * - sensor_offset: Distance offset of the sensor from robot center (in inches)
 * - sensor_angle_offset: Angle offset for the sensor direction (0° = front, 90° = right, 180° = back, 270° = left)
 * - field_half_size: Half the field dimension (distance from center to wall, in inches)

 ** - IMPORTANT NOTE FOR THE USER - **
 * - DO NOT CALL THIS FUNCTION DIRECTLY, use the specific direction functions below instead.
 */
void resetPositionWithSensor(vex::distance& sensor, double s_x, double s_y, double sensor_angle_offset, double field_half_size) {
    double sensorReading = sensor.objectDistance(inches);
  
    if (sensorReading < 0 || sensorReading > 200) {
        return; // Don't reset if reading is clearly garbage
    }
    
    double current_heading_deg = getInertialHeading();
    double sensor_heading_deg = current_heading_deg + sensor_angle_offset;
    
    // Normalize to 0–360
    double heading = fmod(sensor_heading_deg, 360.0);
    if (heading < 0) heading += 360;

    bool resettingX = false;
    double wallAngle = 0;
    double wallSign = 1.0;

    // Identify which wall we are looking at
    if (heading <= 45 || heading >= 315)      { resettingX = false; wallAngle = 0;   wallSign = 1.0; }  // North (+Y)
    else if (heading > 45 && heading <= 135)  { resettingX = true;  wallAngle = 90;  wallSign = 1.0; }  // East (+X)
    else if (heading > 135 && heading <= 225) { resettingX = false; wallAngle = 180; wallSign = -1.0; } // South (-Y)
    else                                      { resettingX = true;  wallAngle = 270; wallSign = -1.0; } // West (-X)

    // Calculate the perpendicular distance from sensor to wall
    double angleErrorRad = (heading - wallAngle) * M_PI / 180.0;
    double perpendicularDistance = sensorReading * cos(angleErrorRad);

    // Convert robot heading to radians
    double thetaRad = current_heading_deg * M_PI / 180.0;
    double rotatedOffset;

    if (resettingX) {
        // Contribution of the sensor's X/Y physical position to the X-axis distance
        rotatedOffset = (s_x * cos(thetaRad)) + (s_y * sin(thetaRad));
    } else {
        // Contribution of the sensor's X/Y physical position to the Y-axis distance
        rotatedOffset = (-s_x * sin(thetaRad)) + (s_y * cos(thetaRad));
    }

    // Final Position Calculation
    double wallToCenter = perpendicularDistance + (wallSign * rotatedOffset);
    double actualPos = wallSign * (field_half_size - wallToCenter);

    if (resettingX) x_pos = actualPos; else y_pos = actualPos;
}

/*
 * resetPositionFront
 * Resets position using the front distance sensor.
 * - sensor: Front distance sensor
 * - sensor_offset: Distance offset of the sensor from robot center (in inches)
 * - field_half_size: Half the field dimension (distance from center to wall, in inches)
 */
void resetPositionFront() {
    resetPositionWithSensor(front_sensor, front_sensor_offsetX, front_sensor_offsetY, 0.0, field_half_size);
}

/*
 * resetPositionBack
 * Resets position using the back distance sensor.
 * Remember to only use these when perpendicular to the wall!
 * - sensor: Back distance sensor 
 * - sensor_offset: Distance offset of the sensor from robot center (in inches)
 * - field_half_size: Half the field dimension (distance from center to wall, in inches)
 */

void resetPositionBack() {
    resetPositionWithSensor(back_sensor, back_sensor_offsetX, back_sensor_offsetY, 180.0, field_half_size);
}
   

/*
 * resetPositionLeft
 * Resets position using the left distance sensor.
 * - sensor: Left distance sensor
 * - sensor_offset: Distance offset of the sensor from robot center (in inches)
 * - field_half_size: Half the field dimension (distance from center to wall, in inches)
 */
void resetPositionLeft() {
    resetPositionWithSensor(left_sensor, left_sensor_offsetX, left_sensor_offsetY, 270.0, field_half_size);
}

/*
 * resetPositionRight
 * Resets position using the right distance sensor.
 * - sensor: Right distance sensor
 * - sensor_offset: Distance offset of the sensor from robot center (in inches)
 * - field_half_size: Half the field dimension (distance from center to wall, in inches)
 */
void resetPositionRight() {
    resetPositionWithSensor(right_sensor, right_sensor_offsetX, right_sensor_offsetY, 90.0, field_half_size);
}

// ============================================================================
// TEMPLATE NOTE
// ============================================================================
// This file is intended as a template for VEX/V5 robotics teams.
// All functions and variables use clear, consistent naming conventions.
// Comments are concise and explain the intent of each section.
// Teams can adapt PID values, drive base geometry, and logic as needed for their robot.
