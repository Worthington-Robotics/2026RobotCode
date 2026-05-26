// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.intake;

/**
 * Stores the intake setpoints
 */
public enum IntakePoses {
  EXTENDED(0.12),
  RETRACTED(1.4),
  HALF(1.2);

  public final double pose;

  /**
   * Stores the intake setpoints
   */
  private IntakePoses(double pose) {
    this.pose = pose;
  }

  public double get() {
    return pose;
  }
}
