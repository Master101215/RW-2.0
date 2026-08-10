#include <string>
#include <cmath>

#include "vex.h"

// --- Global Variables (snake_case) ---
extern bool is_turning;

extern double x_pos, y_pos;
extern double correct_angle;

// --- Function Declarations (lowerCamelCase) ---
void driveChassis(double left_power, double right_power);

double getInertialHeading();
double normalizeTarget(double angle);

#pragma once
#include "vex.h"

// Wall-following drive
void wallDriveForwardLeft(double distance_to_drive_in, double time_limit_msec, double max_output, double target_distance_mm, bool exit);
void wallDriveForwardRight(double distance_to_drive_in, double time_limit_msec, double max_output, double target_distance_mm, bool exit);
void wallDriveBackwardLeft(double distance_to_drive_in, double time_limit_msec, double max_output, double target_distance_mm, bool exit);
void wallDriveBackwardRight(double distance_to_drive_in, double time_limit_msec, double max_output, double target_distance_mm, bool exit);
void turnToAngle(double turn_angle, double time_limit_msec, bool exit = true, double max_output = 100);
void driveTo(double distance_in, double time_limit_msec, bool exit = true, double max_output = 100);
void curveCircle(double result_angle_deg, double center_radius, double time_limit_msec, bool exit = true, double max_output = 100);
void swing(double swing_angle, double drive_direction, double time_limit_msec, bool exit = true, double max_output = 100);
void driveStraight(double distance_in, double time_limit_msec, bool exit, double max_output);

void stopChassis(vex::brakeType type = vex::brake);
void resetChassis();
double getLeftRotationDegree();
double getRightRotationDegree();
void correctHeading();
void trackNoOdomWheel();
void trackXYOdomWheel();
void trackXOdomWheel();
void trackYOdomWheel();
void turnToPoint(double x, double y, int dir, double time_limit_msec);
void moveToPoint(double x, double y, int dir, double time_limit_msec, bool exit = true, double max_output = 100, bool overturn = false);
void boomerang(double x, double y, int dir, double a, double dlead, double time_limit_msec, bool exit = true, double max_output = 100, bool overturn = false);
void resetPositionWithSensor(vex::distance& sensor, double sensor_offset, double sensor_angle_deg, double field_half_size = 72);
void resetPositionFront();
void resetPositionBack();
void resetPositionLeft();
void resetPositionRight();

bool waitForBlock(int targetHue, int timeoutMs);