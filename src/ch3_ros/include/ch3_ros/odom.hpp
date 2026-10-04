#pragma once

struct Odom
{
  Odom() = default;
  Odom(double timestamp, double left_pulse, double right_pulse)
  : timestamp_(timestamp), left_pulse_(left_pulse), right_pulse_(right_pulse) {}

  double timestamp_ = 0.0;
  // Encoder pulses of the left / right wheel over one sampling period
  double left_pulse_ = 0.0;
  double right_pulse_ = 0.0;
};
