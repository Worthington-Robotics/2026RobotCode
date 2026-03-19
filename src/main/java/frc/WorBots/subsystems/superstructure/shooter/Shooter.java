package frc.WorBots.subsystems.superstructure.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.Constants;
import frc.WorBots.Constants.TurretShooterConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.superstructure.shooter.ShooterIO.ShooterIOInputs;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.debug.TunablePIDController;
import frc.WorBots.util.debug.TunablePIDController.TunableProfiledPIDController;

public class Shooter {
  private ShooterIO io;
  private ShooterIOInputs inputs = new ShooterIOInputs();

  private final TunableProfiledPIDController leaderPIDController = new TunableProfiledPIDController("Shooter",
      "Flywheel Leader Controller");

  private final TunablePIDController hoodPIDController = new TunablePIDController("Shooter",
      "Flywheel Hood Controller");

  private final SimpleMotorFeedforward leaderFeedForwardController;
  private double setpointVelocity;
  private double setpointPosition;
  
  private double flySetpointVolts;
  private double hoodSetpointVolts;
  private Drive drive;

  private final double hoodMaxHeading = 0.525;
  private double timeOfFlight = 0;
  private boolean passing = false;

  private double leaderFudgeFactor = 0;
  private double hoodFudgeFactor = 0;
  private boolean autoHoodDown = false;

  // Two control modes, one allows you to provide voltage, the other means that
  // the robot is disabled.
  public enum ControlMode {
    Voltage,
    Setpoint,
    Disabled
  }

  private ControlMode controlMode = ControlMode.Disabled;

  // Publishers
  private final NetworkTable shooter = NetworkTableInstance.getDefault().getTable("Shooter");
  private final StringPublisher controlModePub = shooter.getStringTopic("Control Mode").publish();
  private final DoublePublisher hoodPosePub = shooter.getDoubleTopic("Hood Pose").publish();
  private final DoublePublisher hoodPoseDesiredPub = shooter.getDoubleTopic("Hood Setpoint ").publish();
  private final DoublePublisher shooterSpeedActualPub = shooter.getDoubleTopic("Shooter Speed Actual").publish();
  private final DoublePublisher shooterSpeedDesiredPub = shooter.getDoubleTopic("Shooter Speed Desired").publish();
  private final DoublePublisher flywheelErrorPub = shooter.getDoubleTopic("Flywheel Error").publish();
  private final DoublePublisher hoodErrorPub = shooter.getDoubleTopic("Hood Error").publish();
  private final DoublePublisher hoodFudgePub = shooter.getDoubleTopic("Hood Fudge Factor").publish();
  private final DoublePublisher FlywheelFudgePub = shooter.getDoubleTopic("Flywheel Fudge Factor").publish();

  // a

  // TODO: tune the PIDs, setting the tolerances and feedforward values too.
  /**
   * Creates a Shooter object alongside its respective PIDs.
   * 
   * @param io
   */
  public Shooter(ShooterIO io, Drive drive) {
    this.io = io;
    this.drive = drive;

    leaderPIDController.pid.setTolerance(0.0);
    hoodPIDController.pid.setTolerance(Constants.TurretShooterConstants.HOOD_POS_TOLERANCE);

    leaderFeedForwardController = new SimpleMotorFeedforward(0, 0.021);
    hoodPIDController.setGains(0.0, 0.0, 0.0);

    // Set PID gains if ! in Sim
    if (!Constants.getSim()) {
      leaderPIDController.setGains(0.04, 0, 0);
      leaderPIDController.setConstraints(0, 0);
      hoodPIDController.setGains(3, 0.0, 0);
    }
    // When in Sim
    else {
      leaderPIDController.setGains(1, 0, 0);
      leaderPIDController.setConstraints(5, 5);
      hoodPIDController.setGains(1.1, 0.0, 0.0);
    }
  }

  /***
   * Periodic function. Publishes values for the hood and leader values, alongside
   * setting PIDs for them both.
   */
  public void periodic() {
    io.updateInputs(inputs);
    leaderPIDController.update();
    hoodPIDController.update();

    inputs.hood.publish();
    inputs.leader.publish();
    controlModePub.set(controlMode.toString());
    hoodPosePub.set(inputs.actualHoodPosition);
    hoodPoseDesiredPub.set(setpointPosition);

    shooterSpeedActualPub.set(inputs.actualLeaderVelocityRadPerSec);
    shooterSpeedDesiredPub.set(setpointVelocity);
    flywheelErrorPub.set(setpointVelocity - inputs.actualLeaderVelocityRadPerSec);
    hoodErrorPub.set(setpointPosition - inputs.actualHoodPosition);
    hoodFudgePub.set(hoodFudgeFactor);
    FlywheelFudgePub.set(leaderFudgeFactor);

    StatusPage.reportStatus(StatusPage.SHOOTER_SUBSYSTEM, inputs.leader.isConnected && inputs.follower.isConnected);
    //TODO remove eventually
    SmartDashboard.putBoolean("Near Trench", drive.nearTrench());
    SmartDashboard.putBoolean("Near Blue trench", drive.approachingBlueTrench(.25));
    SmartDashboard.putBoolean("Near Red Trench", drive.approachingRedTrench(.25));
    if (drive.nearTrench()) {
      setpointPosition = 0;
      setHoodPose(setpointPosition);
      autoHoodDown = true;
    } else {
      autoHoodDown = false;
    }
    if (controlMode == ControlMode.Disabled) {
      io.setHoodVolts(0);
      io.setLeaderVolts(0);
    } else if (controlMode == ControlMode.Setpoint) {
      leaderPIDController.pid.setGoal(setpointVelocity);
      double leaderPID = leaderPIDController.pid.calculate(inputs.actualLeaderVelocityRadPerSec);
      double leaderVolts = leaderFeedForwardController.calculateWithVelocities(inputs.actualLeaderVelocityRadPerSec,
          setpointVelocity) + leaderPID;
      double hoodFeedback = hoodPIDController.pid.calculate(inputs.actualHoodPosition, setpointPosition);

      // Home made feedforward
      if (!hoodPIDController.pid.atSetpoint()) {
        hoodFeedback += TurretShooterConstants.HOOD_STATIC_FEEDFORWARD_VOLTAGE
            * ((inputs.actualHoodPosition < setpointPosition) ? 1.0 : -1.0);
      }

      io.setHoodVolts(hoodFeedback);
      io.setLeaderVolts(leaderVolts);
    } else if (controlMode == ControlMode.Voltage) {
      io.setHoodVolts(hoodSetpointVolts);
      io.setLeaderVolts(flySetpointVolts);
    }

    // If the robot is near the trench, set the hood to 0 degrees to prevent hitting
    // the trench.

    StatusPage.reportStatus(StatusPage.SHOOTER_READY, readyToShoot());
  }

  /**
   * @brief run the shot calculator and generated updated setpoints
   */

  /***
   * Disables the robot.
   */
  public void disable() {
    controlMode = ControlMode.Disabled;
    setFlywheelSpeed(0);
  }

  /***
   * Method that returns the setpoint position of the hood.
   * 
   * @return Setpoint Position of the hood.
   */
  public double getDesiredHoodPosition() {
    return setpointPosition;
  }

  /***
   * Method returning the setpoint velocity of the leader.
   * 
   * @return Setpoint Velocity of the leader.
   */
  public double getDesiredLeaderVelocity() {
    return setpointVelocity;
  }

  /***
   * Gets if the hood is at the desired position.
   * 
   * @return true/false.
   */
  public boolean getIsDesiredHoodPosition() {
    return hoodPIDController.pid.atSetpoint();
  }

  /***
   * Gets if the leader is at its desired velocity.
   * 
   * @return true/false.
   */
  public boolean getIsDesiredLeaderVelocity() {
    return leaderPIDController.pid.atGoal();
  }

  /***
   * Makes the shooter execute a set of shooting params
   * 
   * @param params
   */
  public void setShooterMode(ControlMode mode) {
    controlMode = mode;
  }

  /***
   * Sets the flysheel speed
   * 
   * @param speed The speed to set the flywheel to; in m/second
   */
  public void setFlywheelSpeed(double speed) {
    controlMode = ControlMode.Setpoint;
    setpointVelocity = speed + leaderFudgeFactor;
  }

  /***
   * Sets the hood position
   * 
   * @param pose The position to set the hood to
   */
  public void setHoodPose(double pose) {
    controlMode = ControlMode.Setpoint;
    pose = MathUtil.clamp(pose, 0, hoodMaxHeading);
    if (pose == 0){
      setpointPosition = pose;
    } else {
      setpointPosition = pose + hoodFudgeFactor;
    }
  }

  /**
   * Sets the hoods voltage
   * 
   * @param volts the voltage its set to
   */
  public void setHoodVolts(double volts) {
    controlMode = ControlMode.Voltage;
    hoodSetpointVolts = volts;
  }

  /**
   * Sets the voltage driving the fly wheels
   * 
   * @param volts the voltage the flywheels run at
   */
  public void setFlywheelVolts(double volts) {
    controlMode = ControlMode.Voltage;
    flySetpointVolts = volts;
  }

  /**
   * Modifies the fudge factor applied to the hoods desired position
   * 
   * @param increment the amount you want to modify the fudge factor by in radians
   */
  public void modHoodFudgeFactor(double increment) {
    hoodFudgeFactor += increment;
  }

  /**
   * Modifies the fudge factor applied to the flywheels desired position
   * 
   * @param increment
   */
  public void modFlywheelFudgeFactor(double increment) {
    leaderFudgeFactor += increment;
  }

  public double timeOfFlight() {
    return timeOfFlight;
  }

  public boolean flywheelAtSpeed() {
    return Math.abs(inputs.actualLeaderVelocityRadPerSec
        - setpointVelocity) < Constants.TurretShooterConstants.FLYWHEEL_READY_VEL_TOLERANCE;
  }

  public boolean hoodInPosition() {
    return Math.abs(inputs.actualHoodPosition
        - hoodPIDController.pid.getSetpoint()) < Constants.TurretShooterConstants.HOOD_READY_TOLERANCE && !autoHoodDown;
  }

  /**
   * Returns if the shooter is ready to shoot and the shot is valid
   */
  public boolean readyToShoot() {
    return hoodInPosition() && flywheelAtSpeed(); // && isShotValid();
  }

  /**
   * Sets the passing status
   * 
   * @param isPassing is the robot trying to pass
   */
  public void setPassing(boolean isPassing) {
    passing = isPassing;
  }

  public double getHoodPose() {
    return inputs.actualHoodPosition;
  }

  public double getFlyWheelSpeed() {
    return inputs.actualLeaderVelocityRadPerSec;
  }

  /**
   * checks if the shooter is passing
   * 
   * @return true if we are passing
   */
  public boolean getPassing() {
    return passing;
  }
}