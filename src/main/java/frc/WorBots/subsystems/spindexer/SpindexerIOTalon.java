package frc.WorBots.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.WorBots.CanIDs;
import frc.WorBots.Constants;
import frc.WorBots.util.HardwareUtils.TalonSignalsPositional;

public class SpindexerIOTalon implements SpindexerIO{
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SPINDEXER_KS, Constants.SPINDEXER_KV);

  //TODO name canbus
  private TalonFX talon = new TalonFX(CanIDs.SuperStructure.SPINDEXER_ID, "temp");

  private TalonSignalsPositional spinSignal = new TalonSignalsPositional(talon);

  private double voltage = 0.0;

  public SpindexerIOTalon(){}

  @Override
  public void updateInputs(SpindexerIOInputs inputs){
    talon.setVoltage(voltage);
    inputs.active = isActive();
    inputs.jammed = isJammed();
    spinSignal.update(inputs.talon, talon);
  }

  @Override
  public void setVelocity(double vel){
    this.voltage = feedforward.calculate(vel);
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
