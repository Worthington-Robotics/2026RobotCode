package frc.WorBots.subsystems.shooter;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.WorBots.Constants;

public class ShooterIOSim implements ShooterIO {
  //TODO: Update these values in accordance to the actual robot.
  //public SingleJointedArmSim hood = 
  //  new SingleJointedArmSim(null, 0, 0, 0, 0, 0, false, 0, null);
  //public FlywheelSim flyLeader =
  //  new FlywheelSim(null, null, null);

  //Empty constructor   
  public ShooterIOSim(){}

  @Override
  public void updateInputs(ShooterIOInputs inputs){
    inputs.isConnected = true;

    //hood.update(Constants.ROBOT_PERIOD);
    //flyLeader.update(Constants.ROBOT_PERIOD);

    //inputs.actualLeaderVelocityRadPerSec = flyLeader.getAngularVelocityRadPerSec();
    //inputs.actualHoodPosition = hood.getAngleRads();
        
    //inputs.leaderCurrent = flyLeader.getCurrentDrawAmps();
    //inputs.hoodCurrent = hood.getCurrentDrawAmps();
    }

    @Override
    public void setLeaderVolts(double volts){
      //flyLeader.setInputVoltage(volts);
    }

    @Override
    public void setHoodVolts(double volts){
      //hood.setInputVoltage(volts);
    }

    
}
