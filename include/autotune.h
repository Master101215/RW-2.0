#include "vex.h"
#include "motor-control.h"
#include "robot-config.h"
#include <cmath>
#include <vector>

using namespace vex;

// ============================================================================
// AUTO-TUNER (Twiddle, kp + kd only)
// Drives/turns using driveTo function, scores the
// result off encoder/heading feedback, tweaks kp/kd, repeats. Stops early
// once a movement is good enough.
// ============================================================================

const double TUNE_DISTANCE_IN   = 10.0;
const double TUNE_TURN_DEG      = 90.0;
const double TUNE_TIME_LIMIT_MS = 2000;
const double TWIDDLE_TOLERANCE  = 0.01;
const int    TWIDDLE_MAX_ITERS  = 100;

const double W_FINAL_ERROR = 3.0;
const double W_OVERSHOOT   = 2.0;
const double W_TIME        = 0.001;

const double GOOD_ENOUGH_COST = 0.75;

volatile bool   sampler_active = false;
volatile double sampler_start_val = 0;
volatile double sampler_target = 0;
volatile double sampler_peak_overshoot = 0;
volatile bool   sampler_is_turn = false;

bool   tuner_active = false;
bool   tuner_done_early = false;
double tuner_kp = 0;
double tuner_kd = 0;
double tuner_cost = 0;
char   tuner_label[16] = "";
bool   tuner_finished = false;

double driveInches() {
  double avgDeg = (left_chassis.position(deg) + right_chassis.position(deg)) / 2.0;
  return (avgDeg / 360.0) * wheel_distance_in;
}

int samplerTask() {
  while (sampler_active) {
    double current = sampler_is_turn ? inertial_sensor.rotation(degrees) : driveInches();
    double overshoot = current - (sampler_start_val + sampler_target);
    if (overshoot > sampler_peak_overshoot) sampler_peak_overshoot = overshoot;
    this_thread::sleep_for(10);
  }
  return 0;
}

double evaluateDistanceGains(double kp, double kd) {
  distance_kp = kp;
  distance_ki = 0;
  distance_kd = kd;

  resetChassis();
  double startPos = driveInches();

  sampler_is_turn = false;
  sampler_start_val = startPos;
  sampler_target = TUNE_DISTANCE_IN;
  sampler_peak_overshoot = 0;
  sampler_active = true;
  vex::task sampler(samplerTask);

  int t0 = Brain.timer(msec);
  driveTo(TUNE_DISTANCE_IN, TUNE_TIME_LIMIT_MS, true, 100);
  int elapsed = Brain.timer(msec) - t0;

  sampler_active = false;
  sampler.stop();

  double finalError = fabs(driveInches() - (startPos + TUNE_DISTANCE_IN));
  double cost = W_FINAL_ERROR * finalError
              + W_OVERSHOOT * fmax(0.0, sampler_peak_overshoot)
              + W_TIME * elapsed;

  wait(500, msec);
  driveTo(-TUNE_DISTANCE_IN, TUNE_TIME_LIMIT_MS, true, 100);
  wait(500, msec);

  return cost;
}

double evaluateTurnGains(double kp, double kd) {
  turn_kp = kp;
  turn_ki = 0;
  turn_kd = kd;

  inertial_sensor.setRotation(0, degrees);

  sampler_is_turn = true;
  sampler_start_val = 0;
  sampler_target = TUNE_TURN_DEG;
  sampler_peak_overshoot = 0;
  sampler_active = true;
  vex::task sampler(samplerTask);

  int t0 = Brain.timer(msec);
  turnToAngle(TUNE_TURN_DEG, TUNE_TIME_LIMIT_MS, true, 100);
  int elapsed = Brain.timer(msec) - t0;

  sampler_active = false;
  sampler.stop();

  double finalError = fabs(inertial_sensor.rotation(degrees) - TUNE_TURN_DEG);
  double cost = W_FINAL_ERROR * finalError
              + W_OVERSHOOT * fmax(0.0, sampler_peak_overshoot)
              + W_TIME * elapsed;

  wait(500, msec);
  turnToAngle(-TUNE_TURN_DEG, TUNE_TIME_LIMIT_MS, true, 100);
  wait(500, msec);

  return cost;
}

struct TwiddleResult {
  double kp, kd;
  double bestCost;
  bool stoppedEarly;
};

TwiddleResult runTwiddle(double (*evaluate)(double, double), const char* label) {
  std::vector<double> params = {0.2, 0.0}; // start at kp = 0.3, kd = 0
  std::vector<double> dp     = {0.05, 0.02};

  double bestCost = evaluate(params[0], params[1]);

  auto pushStatus = [&](bool early) {
    tuner_kp = params[0];
    tuner_kd = params[1];
    tuner_cost = bestCost;
    tuner_done_early = early;
  };
  pushStatus(false);

  if (bestCost <= GOOD_ENOUGH_COST) {
    pushStatus(true);
    return {params[0], params[1], bestCost, true};
  }

  int iter = 0;
  while ((dp[0] + dp[1]) > TWIDDLE_TOLERANCE && iter < TWIDDLE_MAX_ITERS) {
    bool stoppedEarly = false;
    for (int i = 0; i < 2; i++) {
      params[i] += dp[i];
      double cost = evaluate(params[0], params[1]);

      if (cost < bestCost) {
        bestCost = cost;
        dp[i] *= 1.1;
      } else {
        params[i] -= 2 * dp[i];
        cost = evaluate(params[0], params[1]);

        if (cost < bestCost) {
          bestCost = cost;
          dp[i] *= 1.1;
        } else {
          params[i] += dp[i];
          dp[i] *= 0.9;
        }
      }

      pushStatus(false);

      if (bestCost <= GOOD_ENOUGH_COST) {
        stoppedEarly = true;
        break;
      }
    }
    if (stoppedEarly) {
      pushStatus(true);
      return {params[0], params[1], bestCost, true};
    }
    iter++;
  }

  pushStatus(false);
  return {params[0], params[1], bestCost, false};
}

void runDistanceAutoTune() {
  strcpy(tuner_label, "DIST");
  tuner_active = true;
  tuner_finished = false;

  TwiddleResult r = runTwiddle(evaluateDistanceGains, "Dist");

  distance_kp = r.kp;
  distance_ki = 0;
  distance_kd = r.kd;

  tuner_active = false;
  tuner_finished = true;

  left_chassis.stop(hold);
  right_chassis.stop(hold);
  while (true) {
    wait(100, msec);
  }
}