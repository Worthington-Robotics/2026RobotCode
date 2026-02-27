package frc.WorBots.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.WorBots.Constants;

public class TurretIOSim implements TurretIO {
  public double position = 0.0;
  public double velocity = 0.0;


  private final DCMotorSim turretMotor = 
    new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1.0, 1.0),
       DCMotor.getKrakenX60(1));


  public void setVoltage(double volts) {
    velocity = volts;
  }

  public void updateInputs(TurretIOInputs inputs) {
    position = position + velocity * Constants.ROBOT_PERIOD;
    inputs.turretFusedAngle = position;
  }

  public void resetOffset(){
    position = 0.0;
  }

    //TODO add turretIOSim
}
