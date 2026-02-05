package frc.WorBots.subsystems.intake;

import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface IntakeIO {

  // The inputs for the IntakeIO, these will be updated in other classes.
  public static class IntakeIOInputs {

    TalonInputsPositional intakeMotor = new TalonInputsPositional("Intake", "Intake Motor");
    TalonInputsPositional extendingMotor = new TalonInputsPositional("Intake", "Extending Motor");

    boolean isConnected = false;
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
