// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.drive;

import edu.wpi.first.math.util.Units;

/* Mode for the drivetrain when it stops, enabling things like auto-lock to prevent hits */
public enum StopMode {
  None(new double[4]),
  BlockForwardBackward(
      new double[] {
        Units.degreesToRadians(90.0),
        Units.degreesToRadians(90.0),
        Units.degreesToRadians(90.0),
        Units.degreesToRadians(90.0)
      }),
  BlockLeftRight(new double[4]),
  BlockAll(
      new double[] {
        Units.degreesToRadians(45.0),
        Units.degreesToRadians(-45.0),
        Units.degreesToRadians(-45.0),
        Units.degreesToRadians(45.0)
      }),
  ;

  public final double[] moduleAngles;

  private StopMode(double[] moduleAngles) {
    this.moduleAngles = moduleAngles;
  }
}
