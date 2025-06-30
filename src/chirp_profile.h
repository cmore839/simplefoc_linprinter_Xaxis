#ifndef CHIRP_PROFILE_H
#define CHIRP_PROFILE_H

#include <Arduino.h>

class ChirpProfile {
public:
  float f_start;
  float f_end;
  float sweep_time_sec;
  float amplitude;
  unsigned long t0_us;

  bool enabled = false;
  bool run_once = false;
  bool finished = false;

  // For float control edge detection
  float last_enable_float = 0.0;

  void begin(float f0, float f1, float duration_sec, float amp) {
    f_start = f0;
    f_end = f1;
    sweep_time_sec = duration_sec;
    amplitude = amp;
    enabled = false;
    run_once = true;
    finished = false;
    last_enable_float = 0.0;
    t0_us = micros();
  }

  // Call this every loop to enable/disable via float input
  void setFromFloat(float val) {
    // Rising edge: 0.0 → >0.5 enables sweep
    if (last_enable_float <= 0.5 && val > 0.5) {
      enabled = true;
      finished = false;
      t0_us = micros();  // Reset time on start
    }
    // Falling edge or hold low: disables output
    if (val <= 0.5) {
      enabled = false;
    }

    last_enable_float = val;
  }

  float getPositionRef() {
    if (!enabled || finished) return 0.0;

    unsigned long t_now_us = micros();
    float t_sec = (t_now_us - t0_us) / 1e6;

    if (t_sec > sweep_time_sec) {
      if (run_once) {
        finished = true;
        return 0.0;
      }
      t0_us = t_now_us;
      t_sec = 0;
    }

    float progress = t_sec / sweep_time_sec;
    float f = f_start + (f_end - f_start) * progress;
    return amplitude * sin(2 * PI * f * t_sec);
  }

  float getInstantFrequency() {
    if (!enabled || finished) return 0.0;

    unsigned long t_now_us = micros();
    float t_sec = (t_now_us - t0_us) / 1e6;

    if (t_sec > sweep_time_sec) {
      return run_once ? 0.0 : f_start;
    }

    float progress = t_sec / sweep_time_sec;
    return f_start + (f_end - f_start) * progress;
  }

  bool isRunning() const {
    return enabled && !finished;
  }
};

#endif
