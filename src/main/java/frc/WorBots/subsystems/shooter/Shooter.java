package frc.WorBots.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.WorBots.util.debug.TunablePIDController;
import frc.WorBots.util.debug.TunablePIDController.TunablePIDGains;


public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputs inputs = new ShooterIOInputs();

    private final TunablePIDController followerPID =
        new TunablePIDController(new TunablePIDGains("Shooter", "Flywheel Follower Gains"));

    private final TunablePIDController leaderPID =
        new TunablePIDController(new TunablePIDGains("Shooter", "Flywheel Leader Gains"));

    private final TunablePIDController hoodPID =
        new TunablePIDController(new TunablePIDGains("Shooter", "Hood Gains"));

    private final SimpleMotorFeedforward leaderFeedForward;
    private final SimpleMotorFeedforward hoodFeedForward;


    //Publish the values etc
    public Shooter(ShooterIOInputs inputs){
        this.inputs = inputs;
    }







}
