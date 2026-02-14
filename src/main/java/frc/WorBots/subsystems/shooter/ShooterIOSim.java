package frc.WorBots.subsystems.shooter;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.WorBots.Constants;

public class ShooterIOSim implements ShooterIO {

    public SingleJointedArmSim hood = 
      new SingleJointedArmSim(null, 0, 0, 0, 0, 0, false, 0, null);

    /*   
    public FlywheelSim flyFollower = 
      new FlywheelSim(null, null, null);
    */
    public FlywheelSim flyLeader =
      new FlywheelSim(null, null, null);



    public ShooterIOSim(){}


    public void updateInputs(ShooterIOInputs inputs){

      
      hood.update(Constants.ROBOT_PERIOD);
      flyLeader.update(Constants.ROBOT_PERIOD);

      inputs.isConnected = true;

      inputs.actualLeaderVelocityRadPerSec = flyLeader.getAngularVelocityRadPerSec();
      
      inputs.actualHoodPosition = hood.getAngleRads();

        
      inputs.leaderCurrent = flyLeader.getCurrentDrawAmps();
      
      inputs.hoodCurrent = hood.getCurrentDrawAmps();
        

    }

    

    public void setLeaderVolts(double volts){
      flyLeader.setInputVoltage(volts);
    }

    public void setHoodVolts(double volts){
      hood.setInputVoltage(volts);
    }

    
}
