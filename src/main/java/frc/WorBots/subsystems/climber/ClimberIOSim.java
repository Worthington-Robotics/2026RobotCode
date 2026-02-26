package frc.WorBots.subsystems.climber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ClimberIOSim implements ClimberIO {
  
  private DCMotorSim climbSim;
  private double volts = 0;

  //Climbe Gear Ratio 215.91
  public ClimberIOSim(){
    climbSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 14735394.0/2, 215.91), DCMotor.getKrakenX60(1));
  }

  public void updateInputs(ClimberIOInputs inputs){
    climbSim.update(Constants.ROBOT_PERIOD);

    climbSim.setInputVoltage(volts);

    inputs.motor.isConnected = true;
    inputs.motor.appliedPowerVolts = climbSim.getInputVoltage();
    inputs.motor.currentDrawAmps = climbSim.getCurrentDrawAmps();
    inputs.motor.positionRads = climbSim.getAngularPositionRad();
    inputs.motor.supplyVoltage = climbSim.getInputVoltage();
    inputs.motor.temperatureCelsius = 20;
    inputs.motor.velocityRadsPerSec = climbSim.getAngularVelocityRadPerSec();
  }

  public void setMotorVolts(double volts){
    this.volts = volts;
  }
}
