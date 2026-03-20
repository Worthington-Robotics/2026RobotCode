package frc.WorBots.subsystems.superstructure.turret;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.superstructure.ShotCalculator.ShootingParams;
import frc.WorBots.subsystems.superstructure.turret.TurretIO.TurretIOInputs;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.math.GeneralMath;

public class Turret {
  // TODO figure out how to implement the right time to wrap around

  private final NetworkTable turret = NetworkTableInstance.getDefault().getTable("Turret");
  private final BooleanPublisher absConnectedPub = turret.getBooleanTopic("AbsConnected").publish();
  private final DoublePublisher absAnglePub = turret.getDoubleTopic("AbsAngle").publish();
  private final DoublePublisher relAnglePub = turret.getDoubleTopic("RelAngle").publish();
  private final DoublePublisher fusedAnglePub = turret.getDoubleTopic("FusedAngle").publish();
  private final StringPublisher controlModePub = turret.getStringTopic("ControlMode").publish();
  private final BooleanPublisher atsetpointPub = turret.getBooleanTopic("AtSetPoint").publish();
  private final DoublePublisher debugVoltagePub = turret.getDoubleTopic("DebugVoltage").publish();
  private final DoublePublisher positionPub = turret.getDoubleTopic("Position").publish();
  private final DoublePublisher goalPosePub = turret.getDoubleTopic("Goal Position").publish();
  private final BooleanPublisher lockedPub = turret.getBooleanTopic("Locked").publish();
  private final BooleanPublisher readyPub = turret.getBooleanTopic("Ready").publish();
  private final DoublePublisher errorPub = turret.getDoubleTopic("Error").publish();

  public final TurretIO io;

  private TurretIOInputs inputs = new TurretIOInputs();
  private TurretControlMode controlMode = TurretControlMode.Disabled;

  private Constraints turretConstraints = new Constraints((8 * Math.PI), (12 * Math.PI));
  /// p 3.3 d 0.1 //7.1, 0, 0.3
  private ProfiledPIDController turretFeedBack = new ProfiledPIDController(15, 0, 0.3, turretConstraints);

  public enum TurretControlMode {
    Disabled,
    Position,
    Voltage;
  }

  private boolean turretLocked = false;
  private double debugVoltage = 0;
  private double goalPosition = 0;
  private double goalVelocity = 0;

  public Turret(TurretIO io) {
    this.io = io;
    // turretFeedBack.setTolerance(Constants.TurretShooterConstants.TURRET_POSE_TOLERANCE,
    // Constants.TurretShooterConstants.TURRET_VEL_TOLERANCE);

    turretFeedBack.setTolerance(Constants.TurretShooterConstants.TURRET_POSE_TOLERANCE);

    io.resetOffset();
  }

  public void disable() {
    controlMode = TurretControlMode.Disabled;
  }

  public void periodic() {
    io.updateInputs(inputs);
    StatusPage.reportStatus(StatusPage.TURRET_SUBSYSTEM, inputs.turret.isConnected);
    if (controlMode == TurretControlMode.Disabled) {
      io.setVoltage(0);
    } else {
      if (turretLocked) {
        goalPosition = Constants.TurretShooterConstants.TURRET_LOCK_POSITION;
        controlMode = TurretControlMode.Position;
      }
      if (controlMode == TurretControlMode.Voltage) {
        debugVoltage = MathUtil.clamp(debugVoltage, Constants.TurretShooterConstants.TURRET_MIN_VOLTAGE,
            Constants.TurretShooterConstants.TURRET_MAX_ANGLE);
        io.setVoltage(debugVoltage);
      }
      if (controlMode == TurretControlMode.Position) {
        turretFeedBack.setGoal(new TrapezoidProfile.State(goalPosition, goalVelocity));
        double feedback = turretFeedBack.calculate(inputs.turretFusedAngle);

        if (!turretFeedBack.atSetpoint()) {
          final double KS = (inputs.turretFusedAngle < goalPosition) ? 0.33 : -0.33;
          feedback += KS;
        }

        double volts = feedback; // + feedforward;

        volts = MathUtil.clamp(volts, Constants.TurretShooterConstants.TURRET_MIN_VOLTAGE,
            Constants.TurretShooterConstants.TURRET_MAX_VOLTAGE);

        volts = GeneralMath.hardLimitVelocity(volts, inputs.turretFusedAngle,
            Constants.TurretShooterConstants.TURRET_MIN_ANGLE, Constants.TurretShooterConstants.TURRET_MAX_ANGLE);

        io.setVoltage(volts);
      }

    }

    absConnectedPub.set(inputs.absEncoderConnected);
    absAnglePub.set(inputs.turretAbsAngle);
    relAnglePub.set(inputs.turretRelAngle);
    fusedAnglePub.set(inputs.turretFusedAngle);
    controlModePub.set(controlMode.toString());
    atsetpointPub.set(atGoal());
    debugVoltagePub.set(debugVoltage);
    positionPub.set(getPosition());
    goalPosePub.set(goalPosition);
    lockedPub.set(turretLocked);
    readyPub.set(readyToShoot());
    errorPub.set(goalPosition - inputs.turretFusedAngle);
    inputs.turret.publish();

    StatusPage.reportStatus(StatusPage.TURRET_READY, atGoal());
  }

  private double clampSetpoint(double setpoint) {
    return MathUtil.clamp(setpoint, Constants.TurretShooterConstants.TURRET_MIN_ANGLE,
        Constants.TurretShooterConstants.TURRET_MAX_ANGLE);
  }

  public void stopTurret() {
    setVoltage(0);
  }

  public double getPosition() {
    return inputs.turretFusedAngle;
  }

  /**
   * Method to predict if the turret is about to snap around
   * 
   * @return If the turret is within 3 degrees of snap and heading towards it
   */
  public boolean approachingSnapArround() {
    if (Constants.TurretShooterConstants.TURRET_MAX_ANGLE - Math.abs(inputs.turretFusedAngle) < Units
        .degreesToRadians(3)) {
      // This is just a quick way to check if they have the same sign, if they do it
      // means were heading towards our max
      if (inputs.turret.velocityRadsPerSec * inputs.turretFusedAngle > 0) {
        return true;
      }
    }
    return false;
  }

  public boolean atGoal() {
    return turretFeedBack.atGoal();
  }

  public boolean readyToShoot(){
    return Math.abs(turretFeedBack.getGoal().position - getPosition()) < Constants.TurretShooterConstants.TURRET_READY_TOLERANCE;
  }

  /**
   * Optimizes a setpoint to minimize turret movement
   * 
   * @param positionRads The position to optimize
   * @return The optimized setpoint
   */
  public double optimizeSetpoint(double positionRads) {
    positionRads = MathUtil.angleModulus(positionRads);
    clampSetpoint(positionRads);
    if (controlMode != controlMode.Position) {
      // turretFeedBack.reset(inputs.turretFusedAngle);
    }
    // Bounding logic starts here
    ArrayList<Double> dThetas = new ArrayList<>();
    // Calculates the adjustment for each of the three paths we can take
    double dTheta = positionRads - inputs.turretFusedAngle;
    double dTheta2 = dTheta + Units.degreesToRadians(360);
    double dTheta3 = dTheta - Units.degreesToRadians(360);
    dThetas.add(dTheta);
    dThetas.add(dTheta2);
    dThetas.add(dTheta3);
    SmartDashboard.putNumber("TurretOptimize/dTheta 1", dTheta);
    SmartDashboard.putNumber("TurretOptimize/dTheta 2", dTheta2);
    SmartDashboard.putNumber("TurretOptimize/dTheta 3", dTheta3);

    // Removes any paths that take us beyond our limits
    for (int i = 0; i < dThetas.size(); i++) {
      double endPos = inputs.turretFusedAngle + dThetas.get(i);
      if (endPos > Constants.TurretShooterConstants.TURRET_MAX_ANGLE
          || endPos < Constants.TurretShooterConstants.TURRET_MIN_ANGLE) {
        dThetas.remove(i);
        i--;
      }
    }
    // finds the shortest path
    double shortestPathLength = Double.MAX_VALUE;
    double truePath = 0;

    for (double i : dThetas) {
      if (Math.abs(i) < shortestPathLength) {
        truePath = i;
        shortestPathLength = Math.abs(i);
      }
    }
    if (Math.abs(shortestPathLength) > Math.PI) {
    }

    double output = inputs.turretFusedAngle + truePath;
    controlMode = TurretControlMode.Position;
    SmartDashboard.putNumber("TurretOptimize/output", output);
    return output;
  }

  public void setPosition(double positionRads) {
    controlMode = TurretControlMode.Position;
    // goalPosition = optimizeSetpoint(positionRads);
    positionRads = MathUtil.angleModulus(positionRads);
    positionRads = MathUtil.clamp(positionRads, Constants.TurretShooterConstants.TURRET_MIN_ANGLE,
        Constants.TurretShooterConstants.TURRET_MAX_ANGLE);
    positionRads = optimizeSetpoint(positionRads);
    goalPosition = positionRads;
    goalVelocity = 0;
  }

  public void setVoltage(double volts) {
    debugVoltage = volts;
    if (controlMode != TurretControlMode.Voltage) {
      controlMode = TurretControlMode.Voltage;
    }
  }

 public void setTurretMode(TurretControlMode mode)
 {
   controlMode = mode;
 }

  /**
   * Sets the position of the turret.
   * 
   * @param position The position to set the turret to
   */
  public void setPosition(Rotation2d position) {
    setPosition(position.getRadians());
  }

  /**
   * Sets the position of the turret
   * 
   * @param params    The shooting params to get turret angle from
   * @param robotPose The position of the robot
   */
  public void setPosition(ShootingParams params, Pose2d robotPose) {
    double angle = params.turretAngle().getRadians() - MathUtil.angleModulus(robotPose.getRotation().getRadians());
    angle = MathUtil.angleModulus(angle);
    angle = MathUtil.clamp(angle, Constants.TurretShooterConstants.TURRET_MIN_ANGLE, Constants.TurretShooterConstants.TURRET_MAX_ANGLE);
    setPositionAndVelocity(new Rotation2d(angle), params.turretSpeed());
  }

  /**
   * Sets the goal position and velocity of the turret
   * 
   * @param position   The goal position
   * @param velocity   The goal velocity
   * @param robotAngle The current angle of the robot
   * @implNote Field relative by default
   */
  public void setPositionAndVelocity(Rotation2d position, double velocity) {
    SmartDashboard.putNumber("TurretOptimize/Pre Optimize Position", position.getRadians());
    controlMode = TurretControlMode.Position;
    double setPosition = MathUtil.angleModulus(position.getRadians());
    goalPosition = optimizeSetpoint(setPosition);
    goalVelocity = velocity;
  }

  /**
   * Sets the position of the turret
   * 
   * @param params     The shooting params to get the turret angle from
   * @param robotAngle The angle of the robot
   */
  public void setPosition(ShootingParams params, Rotation2d robotAngle) {
    setFieldRelativePosition(params.turretAngle(), robotAngle);
  }

  /**
   * Sets the position of the turret.
   * 
   * @param position   The field relative angle to set the turret to
   * @param robotAngle The robot's angle
   */
  public void setFieldRelativePosition(Rotation2d position, Rotation2d robotAngle) {
    setPosition(position.getRadians() - MathUtil.angleModulus(robotAngle.getRadians()));
  }

  /**
   * Sets the position of the turret
   * 
   * @param position  The field relative angle to set the turret to
   * @param robotPose The robot's position
   */
  public void setFieldRelativePosition(Rotation2d position, Pose2d robotPose) {
    setPosition(position.getRadians() - MathUtil.angleModulus(robotPose.getRotation().getRadians()));
  }

  /**
   * Sets the position of the turret, minimizing the distance the turret is from 0
   * 
   * @param position The position to set the turret to
   */
  public void setPositionMinDistFromZero(double position) {
    goalPosition = position;
  }

  /**
   * Sets the position of the turret, minimizing the distance the turret is from 0
   * 
   * @param position The position to set the turret to
   */
  public void setPositionMinDistFromZero(Rotation2d position) {
    goalPosition = position.getRadians();
  }

  /**
   * Sets the position of the turret, minimizing the distance the turret is from 0
   * 
   * @param position   The position to set the turret to
   * @param robotAngle The angle of the robot
   */
  public void setPositionMinDistFromZeroFieldRel(Rotation2d position, Rotation2d robotAngle) {
    goalPosition = MathUtil.angleModulus(position.getRadians() - robotAngle.getRadians());
  }

  /**
   * Sets the position of the turret, minimizing the distance the turret is from 0
   * 
   * @param position  The position to set the turret to
   * @param robotPose The position of the robot
   */
  public void setPositionMinDistFromZeroFieldRel(Rotation2d position, Pose2d robotPose) {
    goalPosition = MathUtil.angleModulus(position.getRadians() - robotPose.getRotation().getRadians());
  }

  public void lockTurret() {
    turretLocked = true;
  }

  public void unlockTurret() {
    turretLocked = false;
  }

  public double getDesiredAngle() {
    return turretFeedBack.getGoal().position;
  }
}
