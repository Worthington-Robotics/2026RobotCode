package frc.WorBots.subsystems.spindexer;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.WorBots.Constants;

public class SpindexerIOSim implements SpindexerIO {

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SPINDEXER_KS, Constants.SPINDEXER_KV);

  private FlywheelSim sim = new FlywheelSim(
    LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 
    Constants.SPINDEXER_JKgMETERSSQUARED, Constants.SPINDEXER_GEAR_RATIO),
    DCMotor.getKrakenX60(1));

  private double voltage = 0.0;

  public SpindexerIOSim(){}

  @Override
  public void updateInputs(SpindexerIOInputs inputs){
    System.out.println(voltage);
    sim.setInputVoltage(voltage);
    sim.update(Constants.ROBOT_PERIOD);
    inputs.active = isActive();
    inputs.jammed = isJammed();
    inputs.talon.temperatureCelsius = 40; //arbitrary value, just has to be set bellow max temp
    inputs.talon.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.talon.supplyVoltage = voltage;
    inputs.talon.currentDrawAmps = sim.getCurrentDrawAmps();
    inputs.talon.isConnected = true;
  }

  @Override
  public void setVelocity(double vel){
    this.voltage = feedforward.calculate(vel);
  }

  @Override
  public void stop(){
    this.voltage = 0;
  }

  private boolean isActive(){
    //Checks if the spindexer is rotating more than 15 degrees per second
    if(sim.getAngularVelocity().baseUnitMagnitude() > Units.degreesToRadians(15)){
      return true;
    }
      return false;
  }

  private boolean isJammed(){
    return false;
  }
}
