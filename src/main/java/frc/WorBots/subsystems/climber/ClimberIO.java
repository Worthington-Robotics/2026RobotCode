package frc.WorBots.subsystems.climber;

import frc.WorBots.subsystems.climber.Climber.climberControlMode;
import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface ClimberIO {

  public static class ClimberIOInputs {
    TalonInputsPositional motor = new TalonInputsPositional("Climber", "motor");
  }

  public default void setMotorVolts(double volts){}

  public default void setPosition(double position){}

  public default void updateInputs(ClimberIOInputs inputs){}

  public default void setControlMode(climberControlMode controlMode){}
} 
