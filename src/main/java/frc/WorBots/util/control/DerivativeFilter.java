// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.control;

import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

public class DerivativeFilter {
  private Optional<Double> lastValue = Optional.empty();
  private Optional<Double> lastTimestamp = Optional.empty();
  private double lastDerivative = 0.0;

  private double maxDerivative;

  public DerivativeFilter(double maxDerivative) {
    this.maxDerivative = maxDerivative;
  }

  public double calculate(double value) {
    final double lastValue;
    if (this.lastValue.isPresent()) {
      lastValue = this.lastValue.get();
    } else {
      lastValue = value;
    }
    this.lastValue = Optional.of(value);

    final double timestamp = Timer.getFPGATimestamp();

    final double lastTimestamp;
    if (this.lastTimestamp.isPresent()) {
      lastTimestamp = this.lastTimestamp.get();
    } else {
      lastTimestamp = timestamp;
    }
    this.lastTimestamp = Optional.of(timestamp);

    final double dx = value - lastValue;
    if (dx == 0.0) {
      this.lastDerivative = 0.0;
      return value;
    }

    final double dt = timestamp - lastTimestamp;
    if (dt <= 0.0) {
      this.lastDerivative = Double.POSITIVE_INFINITY;
      return lastValue;
    }

    final double derivative = dx / dt;
    final double newValue;
    if (derivative > maxDerivative) {
      lastDerivative = maxDerivative;
      newValue = lastValue + maxDerivative * dt;
    } else if (derivative < -maxDerivative) {
      lastDerivative = -maxDerivative;
      newValue = lastValue - maxDerivative * dt;
    } else {
      lastDerivative = derivative;
      newValue = value;
    }

    this.lastValue = Optional.of(newValue);
    return newValue;
  }

  public void setMaxDerivative(double maxDerivative) {
    this.maxDerivative = maxDerivative;
  }

  public void reset() {
    lastDerivative = 0.0;
    lastValue = Optional.empty();
    lastTimestamp = Optional.empty();
  }

  public double getLastDerivative() {
    return lastDerivative;
  }
}
