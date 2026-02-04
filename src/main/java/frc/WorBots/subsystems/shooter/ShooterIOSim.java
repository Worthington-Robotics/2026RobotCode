package frc.WorBots.subsystems.shooter;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.WorBots.Constants;

public class ShooterIOSim implements ShooterIO {

    public SingleJointedArmSim hood = 
        new SingleJointedArmSim(null, 0, 0, 0, 0, 0, false, 0, null);

    public FlywheelSim flyfollower = 
        new FlywheelSim(null, null, null);

    public FlywheelSim flyLeader =
        new FlywheelSim(null, null, null);



    public ShooterIOSim(){}


    public void updateInputs(ShooterIOInputs inputs){

        flyFollower.update(Constants.ROBOT_PERIOD);
        hood.update(Constants.ROBOT_PERIOD);
        flyLeader.update(Constants.ROBOT_PERIOD);

        inputs.isConnected = true;

        inputs.leaderVelocityRadPerSec = flyLeader.getAngularVelocityRadPerSec();
        inputs.followerVelocityRadPerSec = flyFollower.getAngularVelocityRadPerSec();
        inputs.hoodVelocityRadPerSec = hood.getAngularVelocityRadPerSec();

        inputs.leaderCurrent = flyLeader.getCurrentDrawAmps();
        inputs.followerCurrent = flyFollower.getCurrentDrawAmps();
        inputs.hoodCurrent = hood.getCurrentDrawAmps();

    }

    public void setFollowerVolts(double volts){
        flyFollower.setInputVoltage(volts);
    }

    public void setLeaderVolts(double volts){
        flyLeader.setInputVoltage(volts);
    }

    public void setHoodVolts(double volts){
        hood.setInputVoltage(volts);
    }
}
