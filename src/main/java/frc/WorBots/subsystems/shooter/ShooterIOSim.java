package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.WorBots.Constants;

public class ShooterIOSim implements ShooterIO {
  //TODO: Update Max angle(Hood), JKgMetersSquared(Fly), Gearing (Fly) to be the real value.
  public SingleJointedArmSim hood = 
   new SingleJointedArmSim(DCMotor.getKrakenX60(1), 79.6, 33.062, Units.inchesToMeters(9.5), 0, Units.degreesToRadians(90), true, 0);
  public FlywheelSim flyLeader =
   new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(2), 100, 1), DCMotor.getKrakenX60(2));

  //Empty constructor   
  public ShooterIOSim(){}

  @Override
  public void updateInputs(ShooterIOInputs inputs){
    inputs.isConnected = true;

    hood.update(Constants.ROBOT_PERIOD);
    flyLeader.update(Constants.ROBOT_PERIOD);

    inputs.actualLeaderVelocityRadPerSec = flyLeader.getAngularVelocityRadPerSec();
    inputs.actualHoodPosition = hood.getAngleRads();
        
    }

    @Override
    public void setLeaderVolts(double volts){
      flyLeader.setInputVoltage(volts);
    }

    @Override
    public void setHoodVolts(double volts){
      hood.setInputVoltage(volts);
    }

    
}
