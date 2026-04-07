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
import frc.WorBots.energy.PowerLogger.SubsystemLog;
import frc.WorBots.subsystems.superstructure.ShotCalculator.ShootingParams;
import frc.WorBots.subsystems.superstructure.turret.TurretIO.TurretIOInputs;
import frc.WorBots.util.debug.StatusPage;

/**
 * A class to represent the robot's turret
 */
public class Turret {
  // Create publishers for logging
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

  private Constraints turretConstraints = new Constraints((4 * Math.PI), (6 * Math.PI));

  public enum TurretControlMode {
    Disabled,
    Position,
    Voltage;
  }

  private boolean turretLocked = false;
  private double debugVoltage = 0;
  private double goalPosition = 0;
  private double goalVelocity = 0;

  /**
   * A class to represent the robot's turret
   * 
   * @param io The TurretIO to use
   */
  public Turret(TurretIO io) {
    this.io = io;
    io.resetOffset();
  }

  /**
   * Disables the turret
   */
  public void disable() {
    controlMode = TurretControlMode.Disabled;
  }

  public void periodic() {
    // Update inputs and report status
    io.updateInputs(inputs);
    StatusPage.reportStatus(StatusPage.TURRET_SUBSYSTEM, inputs.turret.isConnected);

    // Command motors
    if (controlMode == TurretControlMode.Disabled) {
      io.setVoltage(0);
        // volts = MathUtil.clamp(volts, Constants.TurretShooterConstants.TURRET_MIN_VOLTAGE,
        //     Constants.TurretShooterConstants.TURRET_MAX_VOLTAGE);

        // volts = GeneralMath.hardLimitVelocity(volts, inputs.turretFusedAngle,
        //     Constants.TurretShooterConstants.TURRET_MIN_ANGLE, Constants.TurretShooterConstants.TURRET_MAX_ANGLE);

        // io.setVoltage(volts);

        // requestedVoltagePub.set(volts);
        io.setPosition(new TrapezoidProfile.State(goalPosition, goalVelocity));
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
    return Math.abs(goalPosition - getPosition()) < Constants.TurretShooterConstants.TURRET_READY_TOLERANCE;
  }

  /**
   * Returns if the turret is close enough to it's goal to pass
   */
  public boolean readyToPass() {
    return Math.abs(goalPosition - getPosition()) < Constants.TurretShooterConstants.TURRET_READY_PASS_TOLERANCE;
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

  /**
   * Runs the turret at a specified voltage
   * 
   * @param volts The voltage to run the turret at
   */
  public void setVoltage(double volts) {
    debugVoltage = volts;
    if (controlMode != TurretControlMode.Voltage) {
      controlMode = TurretControlMode.Voltage;
    }
  }

  public void setTurretMode(TurretControlMode mode) {
    controlMode = mode;
  }

  /**
   * Sets the field relative position of the turret
   * 
   * @param params    The shooting params to get turret angle from
   * @param robotPose The position of the robot
   */
  public void setPosition(ShootingParams params, Pose2d robotPose) {
    double angle = params.turretAngle().getRadians() - MathUtil.angleModulus(robotPose.getRotation().getRadians());
    setPositionAndVelocity(new Rotation2d(angle), params.turretSpeed());
  }

  /**
   * Sets the robot relative position of the turret
   * 
   * @param position The goal position
   */
  public void setPosition(double position) {
    setPositionAndVelocity(new Rotation2d(position), 0);
  }

  /**
   * Sets the robot relative goal position and velocity of the turret
   * 
   * @param position   The goal position
   * @param velocity   The goal velocity
   * @param robotAngle The current angle of the robot
   */
  public void setPositionAndVelocity(Rotation2d position, double velocity) {
    double positionDouble = MathUtil.angleModulus(position.getRadians());
    positionDouble = clampSetpoint(positionDouble);
    SmartDashboard.putNumber("TurretOptimize/Pre Optimize Position", positionDouble);
    controlMode = TurretControlMode.Position;
    goalPosition = optimizeSetpoint(positionDouble);
    goalVelocity = velocity;
  }

  public double getDesiredAngle() {
    return goalPosition;
  }

  /**
   * Returns the power log containing current and voltage usage for all motors in
   * the turret subsystem
   */
  public SubsystemLog getPowerLog() {
    return new SubsystemLog("Turret", new String[] { "Turret Motor" },
        new double[] { inputs.turret.appliedPowerVolts },
        new double[] { inputs.turret.currentDrawAmps });
  }
}
