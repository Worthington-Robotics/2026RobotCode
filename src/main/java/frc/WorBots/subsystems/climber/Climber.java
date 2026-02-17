package frc.WorBots.subsystems.climber;

import frc.WorBots.subsystems.climber.ClimberIO.ClimberIOInputs;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
  private final ClimberIO io ;
  private final ClimberIOInputs inputs = new ClimberIOInputs();

  private double setPointVoltage = 0.0;
  private double postition;

  private climberControlMode controlMode = climberControlMode.disabled;

  public enum climberControlMode{
    voltage,
    position,
    disabled
  }

  @Override
  public void periodic(){
    io.updateInputs(inputs);
  }

  public Climber(ClimberIO io){
    this.io = io;
  }

  public Double getVolts(){
    return setPointVoltage;
  }

  public void setVolts(double volts){
    setPointVoltage = volts;
  }

  public double getPosition(){
    return postition;
  }

  public void setPosition(double pos){
    postition = pos;
  }

  public void stopMotor(){
    io.setMotorVolts(0.0);
  }

  public void disable(){
    controlMode = climberControlMode.disabled;
  }
}
