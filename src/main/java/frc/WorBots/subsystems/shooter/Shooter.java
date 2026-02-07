package frc.WorBots.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.WorBots.util.debug.TunablePIDController;
import frc.WorBots.util.debug.TunablePIDController.TunablePIDGains;
import frc.WorBots.util.debug.TunablePIDController.TunableProfiledPIDController;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private ShooterIOInputs inputs = new ShooterIOInputs();
<<<<<<< Updated upstream
  //TODO move the PIDs to the ShooterIOTalon
    private final TunablePIDController followerPID =
        new TunablePIDController(new TunablePIDGains("Shooter", "Flywheel Follower Gains"));
=======

    private final TunableProfiledPIDController followerPIDController =
        new TunableProfiledPIDController("Shooter", "Flywheel Follower Controller");
>>>>>>> Stashed changes

    private final TunableProfiledPIDController leaderPIDController =
        new TunableProfiledPIDController("Shooter", "Flywheel Leader Controller");

    private final TunableProfiledPIDController hoodPIDController =
        new TunableProfiledPIDController("Shooter", "Flywheel Hood Controller");

    private final SimpleMotorFeedforward leaderFeedForwardController;
    private final SimpleMotorFeedforward hoodFeedForwardController;
    private double setPointVoltage;

    private enum ControlMode{
      Voltage,
      Disabled
    }

    private ControlMode controlMode = ControlMode.Disabled;
    
    //Publishers
    private final NetworkTable shooter =
      NetworkTableInstance.getDefault().getTable("Shooter");
    private final StringPublisher controlModePub =
      shooter.getStringTopic("Control Mode").publish();
    private final DoublePublisher hoodPose =
      shooter.getDoubleTopic("Hood Pose").publish();
      //hood pose
      //shooter speed -> desired and real

//intake extension and retracts, using set position of the motor
    

    //Publish the values etc
    public Shooter(ShooterIOInputs inputs){
        this.inputs = inputs;

        leaderPIDController.pid.setTolerance(0.0);
        followerPIDController.pid.setTolerance(0.0);
        hoodPIDController.pid.setTolerance(0.0);

        leaderFeedForwardController = new SimpleMotorFeedforward(0.0, 0.0);
        hoodFeedForwardController = new SimpleMotorFeedforward(0.0, 0.0);

        //Set PID gains if ! in Sim
        if(!Constants.getSim()){
          leaderPIDController.setGains(0, 0, 0);
          followerPIDController.setGains(0, 0, 0);
          leaderPIDController.setConstraints(0, 0);
          followerPIDController.setConstraints(0, 0);
        }
        //When in Sim
        else{
          leaderPIDController.setGains(0, 0, 0);
          followerPIDController.setGains(0, 0, 0);
          leaderPIDController.setConstraints(0, 0);
          followerPIDController.setConstraints(0, 0);


        }
    }  

    public void periodic(){
      io.updateInputs(inputs);
      leaderPIDController.update();
      hoodPIDController.update();

      inputs.follower.publish();
      inputs.hood.publish();
      inputs.leader.publish();
      controlModePub.set(controlMode.toString());

      }



    







}
