package frc.WorBots.subsystems.spindexer;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.WorBots.CanIDs;
import frc.WorBots.Constants;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class SpindexerIOTalon implements SpindexerIO{

  //TODO name canbus
  private TalonFX talon = new TalonFX(CanIDs.SuperStructure.SPINDEXER_ID, CanIDs.SuperStructure.CAN_BUS);
  private TalonFX kicker = new TalonFX(CanIDs.SuperStructure.KICKER_ID, CanIDs.SuperStructure.CAN_BUS);

  private TalonSignalsPositional spinSignal = new TalonSignalsPositional(talon);

  private double voltage = 0.0;

  public SpindexerIOTalon(){
    talon.setNeutralMode(NeutralModeValue.Brake);
    kicker.setNeutralMode(NeutralModeValue.Brake);
    kicker.setControl(new Follower(CanIDs.SuperStructure.SPINDEXER_ID, MotorAlignmentValue.Aligned));
  }

  @Override
  public void updateInputs(SpindexerIOInputs inputs){
    talon.setVoltage(voltage);
    inputs.active = isActive();
    inputs.jammed = isJammed();
    //Modifies the talons output velocity to be in radians and be the spindexers velocity
    inputs.spinVelocity = talon.getVelocity().getValueAsDouble() * 2 * Math.PI * Constants.SPINDEXER_GEAR_RATIO;
    spinSignal.update(inputs.talon, talon);
  }

  @Override
  public void setVoltage(double volts){
    this.voltage = volts;
  }

  @Override
  public void stop(){
    this.voltage = 0;
  }

  private boolean isActive(){
    //Checks if the spindexer is rotating more than 15 degrees per second
    //talon.getVelocity returns its value in rotations, the 2Pi is there to convert to radians
    if(talon.getVelocity().getValueAsDouble() * 2 * Math.PI > Units.degreesToRadians(15)){
      return true;
    }
      return false;
  }

  //TODO implement
  private boolean isJammed(){
    return talon.getTorqueCurrent().getValueAsDouble() > Constants.SPINDEXER_STALL_CURRENT && talon.getVelocity().getValueAsDouble() < Constants.SPINDEXER_STALL_SPEED;
  }
}
