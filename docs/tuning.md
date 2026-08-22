---
layout: default
title: Tuning
nav_order: 4
---

# Tuning
{: .no_toc }

Nine PID gains ship as `0`. This page gets them to working values. The autotuner handles the first two, and you hand-tune the rest.
{: .fs-6 .fw-300 }

<details open markdown="block">
  <summary>On this page</summary>
  {: .text-delta }
- TOC
{:toc}
</details>

---

## Do the measurements first

{: .warning }
> **Verify `wheel_distance_in` and `distance_between_wheels` before you tune anything.** Bad geometry looks exactly like bad gains — the robot consistently stops short, and the natural reaction is to raise `kp` until it doesn't. That produces a twitchy controller papering over an arithmetic error, and it falls apart the moment you change `max_output` or drive a different distance.

Test it directly: set a rough `distance_kp` (0.2 is a reasonable guess), run `driveTo(24, 3000)`, and measure with a tape what the robot traveled. If it's off by the same *percentage* at several distances, that's geometry, so fix `wheel_distance_in`. If it's off by a different amount each time, or oscillates, that's tuning.

See [Configuration]({{ site.baseurl }}/configuration#required-measurements) for the formula.

## Tuning order

Work in this order. Each stage depends on the one before it.

1. **`distance_*`**: straight-line driving. The autotuner does this.
2. **`heading_correction_*`**: keeping straight moves straight.
3. **`turn_*`**: turning in place.
4. **Motion feel**: slew rates, `chase_power`, `dlead`.

---

## The autotuner

RW 2.0 ships with a Twiddle optimizer that tunes `distance_kp` and `distance_kd` for you. It drives 10 inches forward and back over and over, scores each attempt, and adjusts.

The binding already exists in `telop()`:

```cpp
controller_1.ButtonY.pressed(runDistanceAutoTune);
```

You do not need to add it.

### Running it

1. Set your ports and chassis measurements in `robot-config.cpp` first.
2. Download the program and start **driver control**.
3. Put the robot on the field or a similar surface with **at least 3 feet of clear space in front and behind it**. It will drive back and forth on its own, repeatedly, for a few minutes.
4. Press **Y**.
5. Watch the controller. It shows `TUNING...` with the current `Kp` and `Kd`; the brain adds the cost score.
6. Wait for `TUNE DONE` or `GOOD VALUES!`, then read the final `Kp` and `Kd`.

### Then copy the values by hand

{: .warning }
> **The autotuner does not save anything.** Its results live in RAM and vanish when the program stops. Write the `Kp` and `Kd` off the screen, put them into `robot-config.cpp`, and re-download:

```cpp
double distance_kp = 0.35, distance_ki = 0, distance_kd = 0.12;
```

It only ever sets `ki` to zero. That's intentional, and a fine place to leave it.

{: .note }
> **The robot stops responding when tuning finishes.** `runDistanceAutoTune()` ends in a deliberate infinite loop with the drive held. That is normal and means it's done — restart the program to get driver control back.

### Two limitations

- **It only tunes distance.** There is no turn autotuner. The code contains an `evaluateTurnGains` function, but nothing calls it and no `runTurnAutoTune()` exists to bind. You tune turn gains by hand, below.
- **It tunes for a 10-inch move.** Gains that suit 10 inches make a good starting point for everything else, but check a long drive (`driveTo(48, 3000)`) afterward and nudge if it overshoots.

### Remove the binding when you're done

Leave `controller_1.ButtonY.pressed(runDistanceAutoTune);` in place and a stray Y press during a match sends the robot driving back and forth on its own. Delete the line once your gains are set.

### How it scores

From [`include/autotune.h`](https://github.com/Master101215/RW-2.0/blob/main/include/autotune.h):

| Constant | Value | Meaning |
|:---------|:------|:--------|
| `TUNE_DISTANCE_IN` | 10.0 | Inches driven per trial |
| `TUNE_TIME_LIMIT_MS` | 2000 | Timeout per trial |
| `TWIDDLE_TOLERANCE` | 0.01 | Stop when the step sizes shrink below this |
| `TWIDDLE_MAX_ITERS` | 100 | Hard iteration cap |
| `GOOD_ENOUGH_COST` | 0.75 | Stop early at this score — shows `GOOD VALUES!` |

The cost function is:

```
cost = 3.0 × |final error|  +  2.0 × overshoot  +  0.001 × elapsed ms
```

Final error carries the most weight, overshoot next, and time almost none, so the tuner chases accuracy and breaks ties on speed. It samples overshoot at 100 Hz through the move rather than reading it once at the end.

Twiddle starts at `kp = 0.2, kd = 0.0` with step sizes `{0.05, 0.02}`. It nudges each parameter up; a better score grows the step by 10%, and a worse one sends it the other direction, shrinking the step by 10% when neither helps.

---

## Tuning by hand

For `turn_*` and `heading_correction_*`, and for refining what the autotuner produced.

### The process

**1. Start with P alone.** Set `ki` and `kd` to zero. Raise `kp` until the robot reaches the target and oscillates around it: visible overshoot, back and forth once or twice.

**2. Back off about 20%.** Sit right at the edge of oscillation.

**3. Add D to damp.** Raise `kd` until the oscillation disappears and the approach is crisp. A good starting ratio is `kd ≈ kp × 3` for turns, less for straight driving. Too much `kd` makes the robot crawl in or jitter. Back off if you hear the motors chattering.

**4. Add I only if you must.** If the robot stops *short* and sits there, a small `ki` (start around `kp / 100`) closes the gap. Otherwise leave `ki` at zero.

{: .tip }
> Change one gain at a time and run the same test move three times before deciding. Battery voltage moves results around enough that a single run will mislead you — and tune on a fresh-ish battery, since gains tuned at 30% charge will be too aggressive at full.

### Turn gains

Test with `turnToAngle(90, 1500)`, then check a small turn (`15°`) and a large one (`180°`).

| Symptom | Fix |
|:--------|:----|
| Doesn't reach the target | Raise `turn_kp` |
| Overshoots and comes back | Raise `turn_kd` |
| Oscillates around the target | Lower `turn_kp`, then raise `turn_kd` |
| Slow, creeping approach | Lower `turn_kd` |
| Good on 90° but short on 15° | Small `turn_ki`, or lower `turn_kd` |

### Heading correction gains

These keep straight moves straight. Tune them *after* your distance gains, with a long test:

```cpp
driveTo(48, 4000);
```

Run it down a taped line. If the robot ends up parallel but offset sideways, you have a mechanical or odometry problem. If it ends up *angled*, raise `heading_correction_kp`.

| Symptom | Fix |
|:--------|:----|
| Veers off heading during the move | Raise `heading_correction_kp` |
| Snakes side to side down the line | Lower `heading_correction_kp`, raise `heading_correction_kd` |
| Buzzes or fights itself when stopped | Lower `heading_correction_kp`, or set `heading_correction = false` |

These gains also hold heading while the robot is stationary, which is why an over-tuned value shows up as a robot that vibrates when it should be still.

---

## Motion feel

Once the PID gains are solid, these settings shape how the robot moves. All are in [`src/robot-config.cpp`](https://github.com/Master101215/RW-2.0/blob/main/src/robot-config.cpp).

### Slew rates

```cpp
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;
```

Max volt change per 10 ms. At `24` the limiter does nothing, since output can swing the full range in a cycle or two. Lower them if the robot wheelies on launch, tips forward when stopping, or burns rubber instead of accelerating. Try `6` to `10` and work up.

Acceleration and deceleration are separate per direction, so you can allow a fast launch while keeping a gentle stop.

### chase_power

```cpp
double chase_power = 2.5;
```

Limits how hard `boomerang` cuts its corner. **Lower it if boomerang paths wander or the robot drifts**, which means the drive is slipping. Raise it for more speed once the path repeats.

### dlead

You pass this per call rather than setting it globally:

```cpp
boomerang(24, 36, 1, 90, 0.5, 3000);
//                        ^^^ dlead
```

Higher = curvier approach. **Never above 0.6.** Start at 0.4 and adjust: raise it if the robot arrives at the right spot but the wrong heading, lower it if the path swings wider than you want.

### dir_change_start / dir_change_end

```cpp
bool dir_change_start = true;
bool dir_change_end = true;
```

Tell the slew limiter whether to expect a direction change at each end of a chained move. `true` is smoother and more accurate, `false` is faster. Set them to `false` once your routine is reliable and you're hunting for tenths.

---

## A tuning session, start to finish

1. Measure and set `wheel_distance_in` and `distance_between_wheels`.
2. Run the autotuner. Copy `Kp` / `Kd` into `distance_kp` / `distance_kd` and re-download.
3. Verify with `driveTo(48, 4000)`. Check both the distance traveled and whether it ended straight.
4. Tune `heading_correction_*` on that same long drive.
5. Tune `turn_*` with `turnToAngle(90, 1500)`, then verify at 15° and 180°.
6. Test `moveToPoint(24, 24, 1, 3000)`, which exercises distance, heading, and odometry together.
7. Test `boomerang` and adjust `dlead`, then `chase_power` if it slips.
8. Delete the Button Y autotuner binding.
9. Re-check on a full battery. Gains tuned low will be aggressive at 100%.