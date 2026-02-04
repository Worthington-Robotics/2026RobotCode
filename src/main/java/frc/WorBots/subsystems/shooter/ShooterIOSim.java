package frc.WorBots.subsystems.shooter;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterIOSim implements ShooterIO {

    public SingleJointedArmSim hood = 
        new SingleJointedArmSim(null, 0, 0, 0, 0, 0, false, 0, null);

    public FlywheelSim flyfollower = 
        new FlywheelSim(null, null, null);

    public FlywheelSim flyLeader =
        new FlywheelSim(null, null, null);

    public ShooterIOSim(){}
















    public void updateInputs(ShooterIOInputs inputs){}

    public void setVolts(double volts){}
}
