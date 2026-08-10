#include "robot-config.h"
#include "vex.h"

void dsrBack(double targetDistance_mm, int timeoutMs, double leniency = 5.0) {
    const double maxVolt = 5.0;
    const double minVolt = 1.0;
    const double slowDist = 150.0; // mm

    auto calcDriveVolt = [&](double error) -> double {
        if (fabs(error) <= leniency) return 0;

        double volt = (fabs(error) / slowDist) * maxVolt;
        if (volt > maxVolt) volt = maxVolt;
        if (volt < minVolt) volt = minVolt;

        return (error > 0) ? -volt : volt;  
        // 🔥 important: negative = backward
    };

    int elapsed = 0;

    while (elapsed < timeoutMs) {
        double leftDist = back_left.objectDistance(mm);
        double rightDist = back_right.objectDistance(mm);

        double leftError = leftDist - targetDistance_mm;
        double rightError = rightDist - targetDistance_mm;

        // Stop condition
        if (fabs(leftError) <= leniency && fabs(rightError) <= leniency) {
            break;
        }

        double leftVolt = calcDriveVolt(leftError);
        double rightVolt = calcDriveVolt(rightError);

        // Apply independently (this auto-corrects angle)
        left_chassis.spin(forward, leftVolt, voltageUnits::volt);
        right_chassis.spin(forward, rightVolt, voltageUnits::volt);

        task::sleep(20);
        elapsed += 20;
    }

    left_chassis.stop();
    right_chassis.stop();
}

void dsrFront(double targetDistance_mm, double targetHeading, int timeoutMs, double leniency = 5.0) {
    const double maxVolt = 5.0;
    const double minVolt = 1.0;
    const double slowDist = 150.0;

    auto calcDriveVolt = [&](double error) -> double {
        if (fabs(error) <= leniency) return 0;

        double volt = (fabs(error) / slowDist) * maxVolt;
        if (volt > maxVolt) volt = maxVolt;
        if (volt < minVolt) volt = minVolt;

        return (error > 0) ? volt : -volt;
    };

    auto angleError = [&](double target, double current) -> double {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    };

    int startTime = Brain.timer(timeUnits::msec);

    while (true) {
        // ⏱️ Timeout check
        if (Brain.timer(timeUnits::msec) - startTime >= timeoutMs) break;

        double rawDist = front_sensor.objectDistance(mm);

        // Safety check
        if (rawDist < 0 || rawDist > 2000) break;

        double currentHeading = getInertialHeading();

        double angleErrDeg = angleError(targetHeading, currentHeading);
        double angleErrRad = angleErrDeg * M_PI / 180.0;

        double correctedDist = rawDist * cos(angleErrRad);
        double driveError = correctedDist - targetDistance_mm;

        // Exit when within tolerance
        if (fabs(driveError) <= leniency) break;

        double driveVolt = calcDriveVolt(driveError);

        left_chassis.spin(forward, driveVolt, voltageUnits::volt);
        right_chassis.spin(forward, driveVolt, voltageUnits::volt);

        task::sleep(20);
    }

    left_chassis.stop();
    right_chassis.stop();
}

void resetAngleBack(double target_offset) {
    const double s_mm = distance_between_sensors * 25.4;
    double d1 = back_left.objectDistance(mm);
    double d2 = back_right.objectDistance(mm);
    if (d1 < 1500 && d2 < 1500) {
        double calculated_skew = atan2((d1 - d2), s_mm) * (180.0 / M_PI);
        double corrected_heading = target_offset + calculated_skew;
        inertial_sensor.setRotation(corrected_heading, degrees);
    }
}