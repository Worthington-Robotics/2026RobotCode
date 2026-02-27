package frc.WorBots.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class ClimberIOTalon implements ClimberIO {
  TalonFX climbMotor = new TalonFX(0);
  TalonSignalsPositional climbMotorSignals;

  public ClimberIOTalon() {
    climbMotorSignals = new TalonSignalsPositional(climbMotor);

  }

  public void setVolts(double volts) {
    climbMotorSignals.setVoltage(climbMotor, volts, 10);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climbMotorSignals.update(inputs.motor, climbMotor);
  }
}