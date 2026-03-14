package frc.WorBots.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.WorBots.CanIDs;
import frc.WorBots.Constants;
import frc.WorBots.util.HardwareUtils;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class ClimberIOTalon implements ClimberIO {
  TalonFX climbMotor = new TalonFX(CanIDs.SuperStructure.CLIMBER_ID, CanIDs.SuperStructure.CAN_BUS);
  TalonSignalsPositional climbMotorSignals;

  public ClimberIOTalon() {
    climbMotorSignals = new TalonSignalsPositional(climbMotor);
    HardwareUtils.setInverted(climbMotor, true);
    climbMotor.setNeutralMode(NeutralModeValue.Brake);
    HardwareUtils.setCurrentLimit(climbMotor, Constants.ClimberConstants.CLIMBER_CURRENT_LIMIT);
    climbMotor.setPosition(0);
  }

  public void setMotorVolts(double volts) {
    climbMotorSignals.setVoltage(climbMotor, volts, 10);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climbMotorSignals.update(inputs.motor, climbMotor);
  }
}