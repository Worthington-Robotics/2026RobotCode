// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.superstructure.turret;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface TurretIO {

  public class TurretIOInputs{
    public TalonInputsPositional turret = new TalonInputsPositional("Turret", "Turret Motor");
    public double turretAbsAngle;
    public double turretRelAngle;
    public double turretFusedAngle;
    public double turretVelocity;
    public double goalAngle;
    public boolean absEncoderConnected = false;
  }

  public void setVoltage(double volts);

  public void updateInputs(TurretIOInputs inputs);

  public void resetOffset();

  public default void setPosition(TrapezoidProfile.State goalState){}
}
