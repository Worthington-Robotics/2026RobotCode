package frc.WorBots.subsystems.superstructure.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.WorBots.Constants;

public class ShooterIOSim implements ShooterIO {
  //TODO: Update Max angle(Hood), JKgMetersSquared(Fly), Gearing (Fly) to be the real value.
  public SingleJointedArmSim hood = 
   new SingleJointedArmSim(DCMotor.getKrakenX60(1), 79.6, 33.062, Units.inchesToMeters(9), 0, Units.degreesToRadians(90), false, 0);
  public FlywheelSim flyLeader =
   new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 40, 1), DCMotor.getKrakenX60(2));

  //Empty constructor   
  public ShooterIOSim(){}

  double leaderVolts = 0;
  double hoodVolts = 0;

  @Override
  public void updateInputs(ShooterIOInputs inputs){
    hood.update(Constants.RobotConstants.ROBOT_PERIOD);
    flyLeader.update(Constants.RobotConstants.ROBOT_PERIOD);

    flyLeader.setInputVoltage(leaderVolts);
    hood.setInputVoltage(hoodVolts);  
    
    inputs.hood.isConnected = true;
    inputs.hood.appliedPowerVolts = hoodVolts;
    inputs.hood.currentDrawAmps = hood.getCurrentDrawAmps();
    inputs.hood.positionRads = hood.getAngleRads();
    inputs.hood.supplyVoltage = 12;
    inputs.hood.temperatureCelsius = 20;
    inputs.hood.velocityRadsPerSec = hood.getVelocityRadPerSec();

    inputs.leader.appliedPowerVolts = leaderVolts;
    inputs.leader.currentDrawAmps = flyLeader.getCurrentDrawAmps();
    inputs.leader.isConnected = true;
    inputs.leader.supplyVoltage = 12;
    inputs.leader.temperatureCelsius = 20;

    inputs.follower.isConnected = true;

    inputs.actualLeaderVelocityRadPerSec = flyLeader.getAngularVelocityRadPerSec();

    inputs.actualHoodPosition = hood.getAngleRads() * Constants.TurretShooterConstants.Hood_GEAR_RATIO;   
    }

    @Override
    public void setLeaderVolts(double volts){
      leaderVolts = volts;
    }

    @Override
    public void setHoodVolts(double volts){
      hoodVolts = volts;
    }

    
}
