// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.subsystems.intake;

import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface IntakeIO {

  // The inputs for the IntakeIO, these will be updated in other classes.
  public static class IntakeIOInputs {

    TalonInputsPositional intakeMotor = new TalonInputsPositional("Intake", "Intake Motor");
    TalonInputsPositional extendingMotor = new TalonInputsPositional("Intake", "Extending Motor");

    boolean isConnected = false;
    // Position of the intake in radians, with 0 being fully out and 90deg being fully up
    double extendPosition;
    double intakeCurrent;
    double extendingCurrent;

  }

  /**
   * Sets the voltage supplied to the intaking motor.
   * 
   * @param volts The amount of volts you wish to supply
   */
  public default void setIntakeMotorVolts(double volts) {
  }

  /**
   * Sets the voltage supplied to the extending motor.
   * 
   * @param volts The amount of volts you wish to supply
   */
  public default void setExtendingMotorVolts(double volts) {
  }

  /**
   * Update intake inputs
   * 
   * @param inputs
   */
  public default void updateInputs(IntakeIOInputs inputs) {
  }

}
