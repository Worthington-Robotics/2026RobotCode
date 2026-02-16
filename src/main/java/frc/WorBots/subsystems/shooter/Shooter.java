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

    private final TunableProfiledPIDController leaderPIDController =
        new TunableProfiledPIDController("Shooter", "Flywheel Leader Controller");

    private final TunableProfiledPIDController hoodPIDController =
        new TunableProfiledPIDController("Shooter", "Flywheel Hood Controller");

    private final SimpleMotorFeedforward leaderFeedForwardController;
    private final SimpleMotorFeedforward hoodFeedForwardController;
    private double setpointVelocity;
    private double setpointPosition;

    //Two control modes, one allows you to provide voltage, the other means that the robot is disabled.
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
    private final DoublePublisher hoodPosePub =
      shooter.getDoubleTopic("Hood Pose").publish();
    private final DoublePublisher hoodPoseDesiredPub = 
      shooter.getDoubleTopic("Hood Setpoint ").publish();
    private final DoublePublisher shooterSpeedActualPub =
      shooter.getDoubleTopic("Shooter Speed Actual").publish();
    private final DoublePublisher shooterSpeedDesiredPub =
      shooter.getDoubleTopic("Shooter Speed Desired").publish();

    

    //TODO: tune the PIDs, setting the tolerances and feedforward values too.  
    /**
     * Creates a Shooter object alongside its respective PIDs.
     * @param inputs
     */
    public Shooter(ShooterIOInputs inputs){
        this.inputs = inputs;

        leaderPIDController.pid.setTolerance(0.0);
        hoodPIDController.pid.setTolerance(0.0);

        leaderFeedForwardController = new SimpleMotorFeedforward(0.0, 0.0);
        hoodFeedForwardController = new SimpleMotorFeedforward(0.0, 0.0);

        //Set PID gains if ! in Sim
        if(!Constants.getSim()){
          leaderPIDController.setGains(0, 0, 0);
          leaderPIDController.setConstraints(0, 0);
          
        }
        //When in Sim
        else{
          leaderPIDController.setGains(0, 0, 0);
          leaderPIDController.setConstraints(0, 0);
          hoodPIDController.setGains(0.0, 0.0, 0.0);
          hoodPIDController.setConstraints(0.0, 0.0);


        }
    }  

    /***
     * Periodic function. Publishes values for the hood and leader values, alongside 
     * setting PIDs for them both.
     */
    @Override
    public void periodic(){
      io.updateInputs(inputs);
      leaderPIDController.update();
      hoodPIDController.update();

    
      inputs.hood.publish();
      inputs.leader.publish();
      controlModePub.set(controlMode.toString());
      hoodPosePub.set(inputs.actualHoodPosition);
      hoodPoseDesiredPub.set(setpointPosition);

      shooterSpeedActualPub.set(0.0);
      shooterSpeedDesiredPub.set(setpointVelocity);

    
      if ( controlMode == ControlMode.Disabled){
        io.setHoodVolts(0);
        io.setLeaderVolts(0);
      }
      else{
        leaderPIDController.pid.setGoal(setpointVelocity);
        double leaderPID = leaderPIDController.pid.calculate(inputs.actualLeaderVelocityRadPerSec);
        leaderFeedForwardController.calculateWithVelocities(inputs.actualLeaderVelocityRadPerSec, setpointVelocity +leaderPID);

        hoodPIDController.pid.setGoal(setpointPosition);
        double hoodPID = hoodPIDController.pid.calculate(inputs.actualHoodPosition);
        hoodFeedForwardController.calculate(hoodPID);

      }
    }

    /***
     * Resets the hood position.
     */
    public void resetHoodPosition(){
      disable();
      io.resetHoodPosition();
    }
    /***
     * Disables the robot.
     */
    public void disable(){
      controlMode = ControlMode.Disabled;
     }

    /***
     * Method that returns the setpoint position of the hood.
     * @return Setpoint Position of the hood.
     */ 
    public double getDesiredHoodPosition(){
      return setpointPosition;
    }
    /***
     * Method returning the setpoint velocity of the leader.
     * @return Setpoint Velocity of the leader.
     */
    public double getDesiredLeaderVelocity(){
      return setpointVelocity;
    }

    /***
     * Gets if the hood is at the desired position.
     * @return true/false.
     */
    public boolean getIsDesiredHoodPosition(){
      return setpointPosition == inputs.actualHoodPosition;
    }

    /***
     * Gets if the leader is at its desired velocity.
     * @return true/false. 
     */
    public boolean getIsDesiredLeaderVelocity(){
      return setpointVelocity == inputs.actualLeaderVelocityRadPerSec;
    }



      }



    








