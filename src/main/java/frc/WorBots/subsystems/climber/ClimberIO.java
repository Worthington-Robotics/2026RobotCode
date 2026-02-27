package frc.WorBots.subsystems.climber;

import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface ClimberIO {

  public static class ClimberIOInputs {
    TalonInputsPositional motor = new TalonInputsPositional("Climber", "Motor");
  }

  public default void setMotorVolts(double volts){}

  public default void updateInputs(ClimberIOInputs inputs){}
} 
