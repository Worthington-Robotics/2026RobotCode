package frc.WorBots.subsystems.superstructure.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.WorBots.Constants;

public class TurretIOSim implements TurretIO {
  public double position = 0.0;
  public double velocity = 0.0;


  private final DCMotorSim turretMotor = 
    new DCMotorSim(
      LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), Constants.TurretShooterConstants.TURRET_MOI, 1.0),
       DCMotor.getKrakenX60(1));


  public void setVoltage(double volts) {
    velocity = volts;
  }

  public void updateInputs(TurretIOInputs inputs) {
    position = position + velocity * Constants.RobotConstants.ROBOT_PERIOD;
    inputs.turretFusedAngle = position * Constants.TurretShooterConstants.TURRET_GEAR_RATIO;
    inputs.turret.isConnected = true;
  }

  public void resetOffset(){
    position = 0.0;
  }

    //TODO add turretIOSim
}
