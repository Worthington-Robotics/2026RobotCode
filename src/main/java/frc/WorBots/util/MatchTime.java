// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * A wrapper utility that gets the time remaining in the current match period, even if no FMS is
 * attached
 */
public class MatchTime {
  private static MatchTime instance = new MatchTime();

  public static MatchTime getInstance() {
    return instance;
  }

  /** The time when the current section was started */
  private double startTime = 0.0;

  /** Amount of time that this section (auto or teleop) will take */
  private double sectionTime = 0.0;

  /** Starts the timer in teleop */
  public void startTeleop() {
    startTime = Timer.getFPGATimestamp();
    sectionTime = 135.0;
  }

  /** Starts the timer in auto */
  public void startAuto() {
    startTime = Timer.getFPGATimestamp();
    sectionTime = 15.0;
  }

  /** Gets the amount of time remaining in the current match period (auto or teleop) */
  public double getTimeRemaining() {
    if (DriverStation.isFMSAttached()) {
      return DriverStation.getMatchTime();
    }

    final double out = sectionTime - (Timer.getFPGATimestamp() - startTime);
    if (out < 0.0) {
      return 0.0;
    } else {
      return out;
    }
  }

  /** Gets the amount of time that has passed in the current match period */
  public double getTimeElapsed() {
    if (DriverStation.isFMSAttached()) {
      return sectionTime - DriverStation.getMatchTime();
    }

    return Timer.getFPGATimestamp() - startTime;
  }
}
