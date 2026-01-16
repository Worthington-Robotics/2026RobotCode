package frc.WorBots.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.GyroIO.GyroIOInputs;
import frc.WorBots.util.OdometryThread;
import frc.WorBots.util.control.DriveFilter;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.GeomUtil;
import frc.WorBots.util.math.PoseEstimator;
import frc.WorBots.util.math.PoseEstimator.*;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class Drive extends SubsystemBase {
  // Constants
  public static final double WHEELBASE = Units.inchesToMeters(10.5);

  /** The drift rate of the robot when driving, in radians per second */
  private static final double DRIFT_RATE = 0.0;

  private static final double STOP_XY_THRESHOLD = Units.inchesToMeters(1.6);
  private static final double STOP_THETA_THRESHOLD = Units.degreesToRadians(1);

  private final Module[] modules = new Module[4];
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroInputs = new GyroIOInputs();

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private final PoseEstimator poseEstimator =
      new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002)); 

  /** The setpoint speeds for the drivetrain */
  private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();

  /** The last field velocity */
  private Twist2d fieldVelocity = new Twist2d();

  /** The last measured robot-relative ChassisSpeeds from odometry */
  private ChassisSpeeds measuredSpeeds;

  /** The last yaw of the gyro, used for delta calculation */
  private Rotation2d lastGyroYaw = new Rotation2d();

  /** The last positions of the modules, used for delta calculations */
  private double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};

  private StopMode stopMode = StopMode.None;

  private DoubleSupplier elevatorHeightSupplier = () -> 1.0;

  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static final String TABLE_NAME = "Drive";
  private final NetworkTable driveTable = instance.getTable(TABLE_NAME);
  private final StructPublisher<ChassisSpeeds> speedSetpointPublisher =
      driveTable.getStructTopic("Speed Setpoint", ChassisSpeeds.struct).publish();
  private final StructPublisher<ChassisSpeeds> goalSetpointPublisher =
      driveTable.getStructTopic("Goal Speed Setpoint", ChassisSpeeds.struct).publish();
  private final StructPublisher<ChassisSpeeds> measuredSpeedsPublisher =
      driveTable.getStructTopic("Measured Speeds", ChassisSpeeds.struct).publish();
  private final DoubleArrayPublisher setpointPublisher =
      driveTable.getDoubleArrayTopic("Module Setpoints").publish();
  private final DoubleArrayPublisher optimizedPublisher =
      driveTable.getDoubleArrayTopic("Optimized Module Setpoints").publish();
  private final DoubleArrayPublisher measuredPublisher =
      driveTable.getDoubleArrayTopic("Measured Module States").publish();
  private final StructPublisher<Pose2d> posePublisher =
      driveTable.getStructTopic("Pose Estimator", Pose2d.struct).publish();
  private final DoublePublisher yawPublisher = driveTable.getDoubleTopic("Gyro/Yaw").publish();
  private final DoublePublisher yawVelocityPublisher =
      driveTable.getDoubleTopic("Gyro/Yaw Velocity").publish();
  private final StringPublisher stopModePublisher =
      driveTable.getStringTopic("Stop Mode").publish();
  private final BooleanPublisher isMeasuredStoppedPublisher =
      driveTable.getBooleanTopic("Is Measured Stopped").publish();

  /**
   * The main swerve drive subsystem
   *
   * @param gyroIO The IO to use for the gyro
   * @param flModule The IO to use for the front left module
   * @param frModule The IO to use for the front right module
   * @param blModule The IO to use for the back left module
   * @param brModule The IO to use for the back right module
   */
  public Drive(
      GyroIO gyroIO, ModuleIO flModule, ModuleIO frModule, ModuleIO blModule, ModuleIO brModule) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModule, 0);
    modules[1] = new Module(frModule, 1);
    modules[2] = new Module(blModule, 2);
    modules[3] = new Module(brModule, 3);
  }

  public void periodic() {
    gyroIO.updateInputs(gyroInputs);

    // Update modules
    for (Module module : modules) {
      module.periodic();
    }

    updateOdometry();

    speedSetpointPublisher.set(setpointSpeeds);
    measuredSpeedsPublisher.set(measuredSpeeds);
    yawPublisher.set(gyroInputs.yawPositionRad);
    yawVelocityPublisher.set(gyroInputs.yawVelocityRadPerSec);
    stopModePublisher.set(stopMode.toString());
    isMeasuredStoppedPublisher.set(isStopped());

    drive();
  }

  /** Drives the drivetrain at the setpoint speeds */
  private void drive() {
    // Update filters for speeds
    final double elevatorRelativePos = elevatorHeightSupplier.getAsDouble();
    driveFilter.limitBasedOnElevator(
        elevatorRelativePos,
        2.1,
        Units.feetToMeters(12.0),
        Units.degreesToRadians(100.0),
        Units.degreesToRadians(500.0));

    final ChassisSpeeds filteredFieldRelative = driveFilter.calculate();
    setpointSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(filteredFieldRelative, getYaw());

    if (DriverStation.isDisabled()) {
      for (Module module : modules) {
        module.stop();
      }
      stop();
      driveFilter.reset();
    } else {
      SwerveModuleState[] setpointStates;
      boolean forceModules = false;

      // Check if we are below the minimum speed and need to stop
      final double magnitude =
          Math.hypot(setpointSpeeds.vxMetersPerSecond, setpointSpeeds.vyMetersPerSecond);
      final boolean isSetpointSlow =
          magnitude < STOP_XY_THRESHOLD
              && Math.abs(setpointSpeeds.omegaRadiansPerSecond) < STOP_THETA_THRESHOLD;
      if (stopMode != StopMode.None) {
        setpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
          setpointStates[i] = new SwerveModuleState(0.0, new Rotation2d(stopMode.moduleAngles[i]));
        }
        forceModules = true;
      } else {
        setpointStates = kinematics.toSwerveModuleStates(setpointSpeeds);
        // Desaturate speeds to ensure we don't go faster than is possible
        SwerveDriveKinematics.desaturateWheelSpeeds(
            setpointStates, getMaxLinearSpeedMetersPerSec());
      }

      setpointPublisher.set(Logger.statesToArray(setpointStates));

      SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        optimizedStates[i] = modules[i].optimizeState(setpointStates[i]);
      }

      optimizedPublisher.set(Logger.statesToArray(optimizedStates));

      // Run the states on the modules
      for (int i = 0; i < 4; i++) {
        modules[i].runState(optimizedStates[i], forceModules);
      }
    }
  }

  /**
   * Updates drivetrain odometry
   *
   * @param timestamp The timestamp when the odometry data was received
   */
  private void updateOdometry() {
    final double startTime = Timer.getFPGATimestamp();

    if (Constants.getSim()) {}

    // Update things for logging and measured velocity that don't need to be as
    // accurate as the odometry

    // Get measured states from modules
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
    }
    measuredPublisher.set(Logger.statesToArray(measuredStates));

    // Update field velocity
    measuredSpeeds = kinematics.toChassisSpeeds(measuredStates);
    final Translation2d linearFieldVelocity =
        new Translation2d(measuredSpeeds.vxMetersPerSecond, measuredSpeeds.vyMetersPerSecond)
            .rotateBy(getRotation());

    // Update field velocity twist
    fieldVelocity =
        new Twist2d(
            linearFieldVelocity.getX(),
            linearFieldVelocity.getY(),
            gyroInputs.connected
                ? gyroInputs.yawVelocityRadPerSec
                : measuredSpeeds.omegaRadiansPerSecond);

    // Actually update the odometry from queues

    OdometryThread.odometryLock.lock();
    ArrayList<Double> timestamps = new ArrayList<>(OdometryThread.timestampQueue.size());
    timestamps.clear();

    while (OdometryThread.timestampQueue.size() > 0) {
      final double timestamp = OdometryThread.timestampQueue.poll();
      timestamps.add(timestamp);
    }
    SmartDashboard.putNumberArray("Timestamps", timestamps.toArray(new Double[0]));

    OdometryThread.odometryLock.unlock();

    // Make sure we don't go out of bounds by only iterating up to the smallest list
    int minSize = timestamps.size();
    for (Module module : modules) {
      minSize = Math.min(minSize, module.getDrivePositionUpdates().size());
      minSize = Math.min(minSize, module.getTurnPositionUpdates().size());
    }
    if (gyroInputs.connected) {
      minSize = Math.min(minSize, gyroInputs.yawPositionUpdates.size());
    } else {
      minSize = 0;
    }

    // Prevent crashing when we don't have drive CAN
    if (minSize > 0) {
      // If there are a lot of updates, only calculate some of them so we don't swamp
      // ourselves
      int modulus = 1;
      if (minSize > 15) {
        modulus = 2;
      } else if (minSize > 30) {
        modulus = 3;
      }

      // Apply the inputs from the modules, gyro, and timestamps for every update we
      // have gotten
      for (int update = 0; update < minSize / modulus; update++) {
        update = update * modulus;

        // Get the deltas of the modules since the last update
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
          final Module module = modules[i];
          final double distance = module.getDrivePositionUpdates().get(update);
          final Rotation2d angle = new Rotation2d(module.getTurnPositionUpdates().get(update));
          wheelDeltas[i] =
              new SwerveModulePosition((distance - lastModulePositionsMeters[i]), angle);
          lastModulePositionsMeters[i] = distance;
        }

        // Do inverse kinematics to get the robot twist
        final Twist2d twist = kinematics.toTwist2d(wheelDeltas);

        // If the gyro is connected, use it's dtheta as it is more accurate
        final Rotation2d gyroYaw = new Rotation2d(gyroInputs.yawPositionUpdates.get(update));
        if (gyroInputs.connected) {
          final double dtheta = gyroYaw.minus(lastGyroYaw).getRadians();
          SmartDashboard.putNumber("Odometry dTheta Error", dtheta - twist.dtheta);
          twist.dtheta = dtheta;
        }
        lastGyroYaw = gyroYaw;

        // Add to pose estimator
        poseEstimator.addDriveDataNoUpdate(timestamps.get(update), twist);
        posePublisher.set(getPose());

        // Update for simulated gyro
        gyroIO.setExpectedYawVelocity(measuredSpeeds.omegaRadiansPerSecond);
      }
    }

    poseEstimator.update();

    final double endTime = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Odometry Time", endTime - startTime);
  }

  /**
   * Adds vision data to the drive subsystem
   *
   * @param updates The vision updates to be added
   */
  public void addVisionData(List<TimestampedVisionUpdate> updates) {
    poseEstimator.addVisionData(updates);
  }

  /**
   * Gets the current rotation of the drive base
   *
   * @return The current yaw of the robot
   */
  public Rotation2d getRotation() {
    return poseEstimator.getLatestPose().getRotation();
  }

  /**
   * Gets the current pose of the robot
   *
   * @return The field-relative robot pose
   */
  public Pose2d getPose() {
    return poseEstimator.getLatestPose();
  }

  /** Enable or disable vision updates on the PoseEstimator */
  public void enableVisionUpdates(boolean enabled) {
    poseEstimator.enableVisionUpdates(enabled);
  }

  /**
   * Gets the current velocity on the field
   *
   * @return Returns the velocity as a twist
   */
  public Twist2d getFieldVelocity() {
    return fieldVelocity;
  }

  /**
   * Gets the setpoint field-relative ChassisSpeeds of the robot
   *
   * @return The speed of the robot
   */
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(setpointSpeeds, getRotation());
  }

  /**
   * Gets the measured robot-relative ChassisSpeeds of the robot
   *
   * @return The speed of the robot
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return measuredSpeeds;
  }

  /**
   * Gets the measured field-relative ChassisSpeeds of the robot
   *
   * @return The speed of the robot
   */
  public ChassisSpeeds getFieldRelativeMeasuredSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(measuredSpeeds, getRotation());
  }

  /**
   * Gets the current yaw velocity
   *
   * @return The yaw velocity in rads per second
   */
  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /**
   * Gets the current yaw
   *
   * @return The yaw as a rotation
   */
  public Rotation2d getYaw() {
    return new Rotation2d(gyroInputs.yawPositionRad);
  }

  /**
   * Sets the pose of the pose estimator, used on starting auto
   *
   * @param pose The pose to be set
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  /**
   * Resets the robot heading
   *
   * @param heading The heading to reset to
   */
  public void resetHeading(Rotation2d heading) {
    gyroIO.resetHeading(heading);
    lastGyroYaw = heading;
    final Pose2d currentPose = poseEstimator.getLatestPose();
    poseEstimator.resetPose(
        new Pose2d(currentPose.getX(), currentPose.getY(), AllianceFlipUtil.apply(heading)));
  }

  /**
   * Gets whether the drive pose is within some margin of a target pose
   *
   * @param target The translation of the target
   * @param maxDistance The margin in meters to say we are close to the target
   * @return Whether the distance to the target <= maxDistance
   */
  public boolean isCloseTo(Translation2d target, double maxDistance) {
    return getPose().getTranslation().getDistance(target) <= maxDistance;
  }

  /**
   * Gets the distance driven of each of the modules in radians
   *
   * @return The distances
   */
  public double[] getModuleDistances() {
    return new double[] {
      modules[0].getPositionRads(),
      modules[1].getPositionRads(),
      modules[2].getPositionRads(),
      modules[3].getPositionRads()
    };
  }

  /**
   * Runs the provided ChassisSpeeds
   *
   * @param speeds The speeds to be run
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Adjust the velocity only once here to reduce calculations
    final ChassisSpeeds adjusted = GeomUtil.driftCorrectChassisSpeeds(speeds, DRIFT_RATE);
    goalSetpointPublisher.set(adjusted);
    final ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(adjusted, getYaw());
    driveFilter.setGoal(fieldRelative);
  }

  /** Stops the drive train by clearing the chassis speeds */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Checks if the robot is measured as stopped */
  public boolean isStopped() {
    final double magnitude =
        Math.hypot(measuredSpeeds.vxMetersPerSecond, measuredSpeeds.vyMetersPerSecond);
    return magnitude < STOP_XY_THRESHOLD
        && Math.abs(measuredSpeeds.omegaRadiansPerSecond) < STOP_THETA_THRESHOLD;
  }

  /** Sets the stop mode for the drivetrain, changing locking behavior */
  public void setStopMode(StopMode mode) {
    stopMode = mode;
  }

  /**
   * Sets the supplier for the height of the elevator. The supplier must return relative values from
   * 0 to 1
   */
  public void setElevatorHeightSupplier(DoubleSupplier supplier) {
    this.elevatorHeightSupplier = supplier;
  }

  /**
   * Gets the module translations relative to the robot's center
   *
   * @return The translations
   */
  public Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(WHEELBASE, WHEELBASE),
      new Translation2d(WHEELBASE, -WHEELBASE),
      new Translation2d(-WHEELBASE, WHEELBASE),
      new Translation2d(-WHEELBASE, -WHEELBASE)
    };
  }

  /**
   * Returns the maximum linear speed (free speed) that the drive train can physically attain
   *
   * @return The value in meters per second
   */
  public double getMaxLinearSpeedMetersPerSec() {
    return Units.feetToMeters(25.5);
  }

  public double getCurrentMaxVelocity() {
    return driveFilter.getMaxVelocity();
  }

  public double getCurrentMaxAcceleration() {
    return driveFilter.getMaxAcceleration();
  }

  public double getCurrentMaxRotationalVelocity() {
    return driveFilter.getMaxRotationalVelocity();
  }

  public double getCurrentMaxRotationalAcceleration() {
    return driveFilter.getMaxRotationalAcceleration();
  }
}
