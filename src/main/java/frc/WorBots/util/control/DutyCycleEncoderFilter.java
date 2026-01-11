// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.control;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.Optional;

/**
 * A utility class that filters out the readings from a duty cycle encoder to throw away the jumps
 * that are produced when it is disconnected. The DutyCycleEncoder.isConnected() on it's own isn't
 * fast enough to catch bad readings when they happen, so this class hopes to remedy that by
 * detecting large spikes in the encoder reading, entering a "safe mode" until normal values start
 * coming back again
 */
public class DutyCycleEncoderFilter {
  /*
   * The maximum difference between the last update before we enter or stay in
   * safe mode
   */
  private final double MAX_DIFFERENCE = Units.radiansToRotations(0.5);
  /* The minimum difference between the last update we need to leave safe mode */
  private final double MIN_DIFFERENCE = 1e-3;
  /*
   * The threshold that if we are beyond jumps won't count since we are probably
   * just wrapping the absolute encoder. Recorded as a threshold
   */
  private final double WRAPAROUND_THRESHOLD = 0.95;

  private Optional<Double> lastValue = Optional.empty();
  private boolean inSafeMode = false;

  public Optional<Double> calculate(DutyCycleEncoder encoder) {
    if (!encoder.isConnected()) {
      return Optional.empty();
    }

    final double newValue = encoder.get();
    final double difference;
    final boolean lastValueWasAtLimit;
    if (lastValue.isPresent()) {
      difference = Math.abs(newValue - lastValue.get());
      lastValueWasAtLimit = isAtEdgeOfRange(lastValue.get());
    } else {
      difference = 0.0;
      lastValueWasAtLimit = false;
    }
    lastValue = Optional.of(newValue);

    if (inSafeMode) {
      // Escape safe mode if we are back to normal difference readings
      if (difference > MIN_DIFFERENCE && difference < MAX_DIFFERENCE) {
        inSafeMode = false;
      } else {
        return Optional.empty();
      }
    } else {
      // Enter safe mode if the difference is too large, and wasn't caused by
      // wraparound
      final boolean wrappedAround = lastValueWasAtLimit && isAtEdgeOfRange(newValue);
      if (!wrappedAround && difference > MAX_DIFFERENCE) {
        inSafeMode = true;
        return Optional.empty();
      }
    }

    return Optional.of(newValue);
  }

  public boolean isInSafeMode() {
    return inSafeMode;
  }

  private boolean isAtEdgeOfRange(double value) {
    return value < (1.0 - WRAPAROUND_THRESHOLD) || value > WRAPAROUND_THRESHOLD;
  }
}
