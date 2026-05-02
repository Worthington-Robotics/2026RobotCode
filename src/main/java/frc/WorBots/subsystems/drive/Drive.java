package frc.WorBots.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.FieldConstants;
import frc.WorBots.RobotContainer;
import frc.WorBots.energy.PowerLogger.SubsystemLog;
import frc.WorBots.subsystems.drive.GyroIO.GyroIOInputs;
import frc.WorBots.util.OdometryThread;
import frc.WorBots.util.control.DriveFilter;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.math.AllianceFlipUtil;
import frc.WorBots.util.math.GeomUtil;
import frc.WorBots.util.math.PoseEstimator;
import frc.WorBots.util.math.PoseEstimator.TimestampedVisionUpdate;

public class Drive extends SubsystemBase {
  private final Module[] modules = new Module[4];
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroIOInputs = new GyroIOInputs();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private DriveFilter filter = new DriveFilter(Constants.DriveConstants.DRIVE_MAX_VELOCITY,
      Constants.DriveConstants.DRIVE_MAX_ACCELERATION, Constants.DriveConstants.DRIVE_MAX_ROTATIONAL_VELOCITY,
      Constants.DriveConstants.DRIVE_MAX_ROTATION_ACCELERATION);

  /* Setpoint speed from the drive filter, robot relative */
  private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();

  /* Measured speeds from odom, robot relative */
  private ChassisSpeeds measuredSpeeds = new ChassisSpeeds();
  private ChassisSpeeds acceleration = new ChassisSpeeds();
  private ChassisSpeeds lastAcceleration = new ChassisSpeeds();

  private Rotation2d lastGyroYaw = new Rotation2d();

  private double[] lastModulePositionMeters = new double[4];
  private double maxSpeed = Constants.DriveConstants.DRIVE_MAX_VELOCITY;

  private StopMode stopMode = StopMode.None;

  private PoseEstimator poseEstimator = new PoseEstimator(VecBuilder.fill(0.01, 0.01, 0.0002));

  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static final String TABLE_NAME = "Drive";
  private final NetworkTable driveTable = instance.getTable(TABLE_NAME);

  private final StructPublisher<ChassisSpeeds> speedSetpointPublisher = driveTable
      .getStructTopic("Speed Setpoint", ChassisSpeeds.struct).publish();
  private final StructPublisher<ChassisSpeeds> goalSetpointPublisher = driveTable
      .getStructTopic("Goal Speed Setpoint", ChassisSpeeds.struct).publish();
  private final StructPublisher<ChassisSpeeds> measuredSpeedPublisher = driveTable
      .getStructTopic("Measured Speed", ChassisSpeeds.struct).publish();
  private final DoubleArrayPublisher moduleSetpointPublisher = driveTable.getDoubleArrayTopic("Module Setpoints")
      .publish();
  private final DoubleArrayPublisher moduleOptimizedPublisher = driveTable.getDoubleArrayTopic("Module Optimal")
      .publish();
  private final DoubleArrayPublisher moduleMeasuredPublisher = driveTable.getDoubleArrayTopic("Module Measured")
      .publish();
  private final StructPublisher<Pose2d> posePublisher = driveTable.getStructTopic("Pose", Pose2d.struct).publish();
  private final DoublePublisher gyroPublisher = driveTable.getDoubleTopic("Gyro Yaw").publish();
  private final DoublePublisher gyroVelocityPublisher = driveTable.getDoubleTopic("Gyro Velocity").publish();
  private final StringPublisher stopModePublisher = driveTable.getStringTopic("Stop Mode").publish();
  private final BooleanPublisher measuredStopPublisher = driveTable.getBooleanTopic("Measured Stop").publish();

  /**
   * Constructor for the Drive Subsytem
   * 
   * @param gyro     The interface for a gyro that tracks the drivetrains
   *                 orientation
   * @param flModule The interface of the drivetrain's front left module
   * @param frModule The interface of the drivetrain's front right module
   * @param blModule The interface of the drivetrain's back left module
   * @param brModule The interface of the drivetrain's back right module
   */

  public Drive(GyroIO gyro, ModuleIO flModule, ModuleIO frModule, ModuleIO blModule, ModuleIO brModule) {
    gyroIO = gyro;
    modules[0] = new Module(flModule, 0);
    modules[1] = new Module(frModule, 1);
    modules[2] = new Module(blModule, 2);
    modules[3] = new Module(brModule, 3);
    StatusPage.reportStatus(StatusPage.DRIVE_SUBSYSTEM, true);
  }

  public void periodic() {
    // updates gyro data
    gyroIO.updateInputs(gyroIOInputs);

    // runs the modules
    for (Module a : modules) {
      a.periodic();
    }

    // passes new information to the odometry and pose estimator
    updateOdometry();

    // publishes information to the drive table
    speedSetpointPublisher.set(setpointSpeeds);
    gyroPublisher.set(gyroIOInputs.yawPositionRad);
    gyroVelocityPublisher.set(gyroIOInputs.yawVelocityRadPerSec);
    stopModePublisher.set(stopMode.toString());
    measuredStopPublisher.set(isStopped());

    // Reports to Status page
    StatusPage.reportStatus(StatusPage.GYROSCOPE, gyroIOInputs.connected);

    // makes the robot move
    drive();
  }

  private void drive() {
    SwerveModuleState[] setpointStates;
    boolean forceModules = false;

    final ChassisSpeeds filteredFieldRelative = filter.calculate();
    setpointSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(filteredFieldRelative, getYaw());

    // if we're disabled the modules are all stoped, the goal speed is set to 0, and
    // the filter is reset
    if (DriverStation.isDisabled()) {
      for (Module a : modules) {
        a.stop();
      }
      stop();
      filter.reset();
    } else {
      if (isStopped()) {
        setpointStates = setStop();
        forceModules = true;
      } else {
        setpointStates = kinematics.toSwerveModuleStates(setpointSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.DriveConstants.DRIVE_MAX_VELOCITY);
      }

      moduleSetpointPublisher.set(Logger.statesToArray(setpointStates));

      SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
      for (int i = 0; i < 4; i++) {
        optimizedStates[i] = modules[i].optimizeState(setpointStates[i]);
      }

      moduleOptimizedPublisher.set(Logger.statesToArray(optimizedStates));

      for (int i = 0; i < 4; i++) {
        modules[i].runState(optimizedStates[i], forceModules);
      }
    }

  }

  private Translation2d[] getModuleTranslations() {
    return Constants.DriveConstants.DRIVE_MODULE_OFFSETS;
  }

  public void setDriveMaxSpeed(double maxSpeed)
  {
    this.maxSpeed = maxSpeed;
  }

  public double getDriveMaxSpeed()
  {
    return maxSpeed;
  }

  /**
   * Tells the robot to move at a velocity of 0, does not directly use stop modes
   */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Moves all modules to form a x causing the robot to hardstop in place
   */
  public SwerveModuleState[] setStop() {
    SwerveModuleState[] setpointStates = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      setpointStates[i] = new SwerveModuleState(0.0, modules[i].getAngle());
    }
    return setpointStates;
  }

  /**
   * Updates the drive filter with the new field relative speed the robot should
   * be driving at
   * 
   * @param speeds the robot relative speed being requested of the robot
   */
  public void runVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds adjusted = GeomUtil.driftCorrectChassisSpeeds(speeds, Constants.DriveConstants.DRIVE_DRIFT_RATE);
    goalSetpointPublisher.set(adjusted);

    // Calculates a field relative velocity as if we're on blue, then flips it to
    // red if nessesary
    ChassisSpeeds fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getYaw());
    // ChassisSpeeds allianceRel = AllianceFlipUtil.flipSpeeds(fieldRel);
    filter.setGoal(fieldRel);
  }

  /**
   * Checks our linear and rotational velocity to see if the robot is physicially
   * stopped
   */
  public boolean isStopped() {
    double magnitude = Math.hypot(setpointSpeeds.vxMetersPerSecond, setpointSpeeds.vyMetersPerSecond);
    return magnitude < Constants.DriveConstants.DRIVE_STOP_XY_THRESHOLD &&
        Math.abs(setpointSpeeds.omegaRadiansPerSecond) < Constants.DriveConstants.DRIVE_THETA_THRESHOLD;
  }

  /**
   * Returns the robots rotation acording to the drivetrain's gyroscope
   */
  public Rotation2d getYaw() {
    return new Rotation2d(gyroIOInputs.yawPositionRad);
  }

  /**
   * Returns the robots rotational velocity acording to the drivetrain's gyroscope
   */
  public Rotation2d getYawVelocity() {
    return new Rotation2d(gyroIOInputs.yawVelocityRadPerSec);
  }

  /**
   * Returns the robot relative velocity, mostly for pathPlanner
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return ChassisSpeeds.fromFieldRelativeSpeeds(filter.calculate(), getYaw());
  }

  /**
   * Returns the robot relative velocity, mostly for pathPlanner
   */
  public ChassisSpeeds getFieldRelativeSetpointSpeeds() {
    return filter.calculate();
  }

  /**
   * Passes new drive information to the Odometry threat and pose estimator
   */
  public void updateOdometry() {
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      measuredStates[i] = modules[i].getState();
    }
    moduleMeasuredPublisher.set(Logger.statesToArray(measuredStates));

    measuredSpeeds = kinematics.toChassisSpeeds(measuredStates);

    OdometryThread.odometryLock.lock();
    ArrayList<Double> timestamps = new ArrayList<>(OdometryThread.timestampQueue.size());
    timestamps.clear();

    while (OdometryThread.timestampQueue.size() > 0) {
      final double timestamp = OdometryThread.timestampQueue.poll();
      timestamps.add(timestamp);
    }

    OdometryThread.odometryLock.unlock();

    int minSize = timestamps.size();
    for (Module module : modules) {
      minSize = Math.min(minSize, module.getDrivePositionUpdates().size());
      minSize = Math.min(minSize, module.getTurnPositionUpdates().size());
    }
    if (gyroIOInputs.connected) {
      minSize = Math.min(minSize, gyroIOInputs.yawPositionUpdates.size());
    } else {
      minSize = 0;
    }

    int modulus = 1;

    if (minSize > 0) {
      if (minSize > 15) {
        modulus = 2;

      } else if (minSize > 30) {
        modulus = 3;
      }
    }
    int update = 0;
    for (update = 0; update < minSize / modulus; update++) {
      update = update * modulus;

      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];

      for (int i = 0; i < 4; i++) {
        final Module module = modules[i];
        final double distance = module.getDrivePositionUpdates().get(update);
        final Rotation2d angle = new Rotation2d(module.getTurnPositionUpdates().get(update));

        wheelDeltas[i] = new SwerveModulePosition((distance - lastModulePositionMeters[i]), angle);
        lastModulePositionMeters[i] = distance;
      }

      final Twist2d twist = kinematics.toTwist2d(wheelDeltas);

      final Rotation2d gyroYaw = new Rotation2d(gyroIOInputs.yawPositionUpdates.get(update));
      if (gyroIOInputs.connected) {
        final double dtheta = gyroYaw.minus(lastGyroYaw).getRadians();

        twist.dtheta = dtheta;
      }

      lastGyroYaw = gyroYaw;

      poseEstimator.addDriveDataNoUpdate(timestamps.get(update), twist);
      
      gyroIO.setExpectedYawVelocity(measuredSpeeds.omegaRadiansPerSecond);
    }
    measuredSpeedPublisher.set(measuredSpeeds);
    poseEstimator.update();
    posePublisher.set(getPose());

    // final double endTime = Timer.getFPGATimestamp();
    // SmartDashboard.putNumber("Odom Time", endTime - startTime);
  }

  /**
   * sets the robots pose and rotation
   * 
   * @return
   */
  public void resetPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  /**
   * Returns the robots rotation according to the pose estimator
   */
  public Rotation2d getRotation() {
    return poseEstimator.getLatestPose().getRotation();
  }

  /**
   * Returns the robots position according to the pose estimator
   */
  public Pose2d getPose() {
    return poseEstimator.getLatestPose();
  }

  /**
   * Returns the robot's measured speeds
   * @return Returns robot relative measured speeds
   */
  public ChassisSpeeds getMeasuredSpeeds() {
    return measuredSpeeds;
  }

  /**
   * Returns the robot's field relative measured speed
   * @return Field relative measured speeds
   */
  public ChassisSpeeds getFieldrelativeMeasuredSpeeds(){
    return ChassisSpeeds.fromRobotRelativeSpeeds(measuredSpeeds, getRotation());
  }

  public void addVisionUpdate(List<TimestampedVisionUpdate> update) {
    poseEstimator.addVisionData(update);
  }

  /*
   * line segment that represents the path the robot will take if it continues on
   * its setpoint speeds for a variable amount of time,
   * used for checking if the robot is approaching the trenches or zones
   */
  public Translation2d[] appliedChassisPath(double seconds) {
    Translation2d finalPose = GeomUtil
        .clampTranslation(GeomUtil.applyChassisSpeeds(getPose(), getFieldRelativeSetpointSpeeds(), seconds)
            .getTranslation());
    return new Translation2d[] { getPose().getTranslation(), finalPose };
  }

  public boolean inRedZone() {
    return GeomUtil.translation2dInBoundingBox(getPose().getTranslation(), FieldConstants.redZone);
  }

  public boolean inBlueZone() {
    return GeomUtil.translation2dInBoundingBox(getPose().getTranslation(), FieldConstants.blueZone);
  }

  public boolean inNeutralZone() {
    return GeomUtil.translation2dInBoundingBox(getPose().getTranslation(), FieldConstants.neutralZone);
  }

  public boolean inOurAllianceZone(){
    if(AllianceFlipUtil.shouldFlip()){
      return inRedZone();
    } else {
      return inBlueZone();
    }
  }

  public boolean shouldStartPassing(){
    if(AllianceFlipUtil.shouldFlip()){
      return getPose().getX() < Units.inchesToMeters(469.11) - Constants.TurretShooterConstants.CHANGE_TARGET_MARGIN;
    } else {
      return getPose().getX() > Units.inchesToMeters(182.11) + Constants.TurretShooterConstants.CHANGE_TARGET_MARGIN; 
    }
  }

  public boolean shouldStartScoring(){
    if(AllianceFlipUtil.shouldFlip()){
      return getPose().getX() > Units.inchesToMeters(469.11) + Constants.TurretShooterConstants.CHANGE_TARGET_MARGIN;
    } else {
      return getPose().getX() < Units.inchesToMeters(182.11) - Constants.TurretShooterConstants.CHANGE_TARGET_MARGIN; 
    }
  }

  /**
   * Checks if the robot is approaching the red zone by checking if the line
   * segment passes the bounding box of the red zone.
   * 
   * @return if the robot is approaching the red zone
   */
  public boolean approachingRedZone(double seconds) {
    return GeomUtil.doesLinePassThroughArea(appliedChassisPath(seconds), FieldConstants.redZone);
  }

  /**
   * Checks if the robot is approaching the blue zone by checking if the line
   * segment passes the bounding box of the blue zone.
   * 
   * @return if the robot is approaching the blue zone
   */
  public boolean approachingBlueZone(double seconds) {
    return GeomUtil.doesLinePassThroughArea(appliedChassisPath(seconds), FieldConstants.blueZone);
  }

  /**
   * Checks if the robot is approaching the neutral zone by checking if the line
   * segment passes the bounding box of the neutral zone.
   * 
   * @return if the robot is approaching the neutral zone
   */
  public boolean approachingNeutralZone(double seconds) {
    return GeomUtil.doesLinePassThroughArea(appliedChassisPath(seconds), FieldConstants.neutralZone);
  }

  /**
   * Checks if the robot is approaching either the top or bottom red trenches by
   * checking if the line segment passes the bounding boxes of the red trenches.
   * 
   * @return if the robot is approaching a red trench
   */
  public boolean approachingRedTrench(double seconds) {
    return GeomUtil.doesLinePassThroughArea(appliedChassisPath(seconds), FieldConstants.redBottomTrench) ||
        GeomUtil.doesLinePassThroughArea(appliedChassisPath(seconds), FieldConstants.redTopTrench);

  }

  /**
   * Checks if the robot is approaching either the top or bottom blue trenches by
   * checking if the line segment passes the bounding boxes of the blue trenches.
   * 
   * @return if the robot is approaching a blue trench
   */
  public boolean approachingBlueTrench(double seconds) {
    return GeomUtil.doesLinePassThroughArea(appliedChassisPath(seconds), FieldConstants.blueBottomTrench) ||
        GeomUtil.doesLinePassThroughArea(appliedChassisPath(seconds), FieldConstants.blueTopTrench);
  }

  public static double chassisSpeedsLength = .25;

  /**
   * First checks what zone robot is in, then checks if the robot is approaching
   * the trenches in that zone.
   * 
   * @return if the robot is approaching a trench
   */
  public boolean nearTrench() {
    // if (inBlueZone()) {
    //   return approachingBlueTrench(chassisSpeedsLength);
    // }

    // else if (inRedZone()) {
    //   return approachingRedTrench(chassisSpeedsLength); // tune these values
    // }

    // else {
      return approachingRedTrench(chassisSpeedsLength) || approachingBlueTrench(chassisSpeedsLength);
    // }
  }

  public void resetYaw(){
    resetYaw(new Rotation2d());
  }

  public void resetYaw(Rotation2d rotation){
     gyroIO.resetHeading(rotation);
    lastGyroYaw = rotation;
    final Pose2d currentPose = poseEstimator.getLatestPose();
    poseEstimator.resetPose(
        new Pose2d(currentPose.getX(), currentPose.getY(), AllianceFlipUtil.apply(rotation)));
    setDriveZeroOffset();
  }

  /**
   * Changes the rotational zero drive controller uses for converting to field relative
   * Relies on having a acurate pose.
   */
  public void setDriveZeroOffset(){
    RobotContainer.driveController.resetDriveRotation(getRotation(), getYaw());
  }

  /**
   * Returns the robot's field relative acceleration
   */
  public ChassisSpeeds getAcceleration(){
    // acceleration = measuredSpeeds.minus(lastMeasuredSpeeds).times(Constants.RobotConstants.ROBOT_FREQUENCY);
    // double x = accelerationFilterX.calculate(acceleration.vxMetersPerSecond);
    // x = MathUtil.clamp(x, -7, 7);
    // double y = accelerationFilterY.calculate(acceleration.vyMetersPerSecond);
    // y = MathUtil.clamp(y, -7, 7);
    // acceleration = new ChassisSpeeds(x, y, acceleration.omegaRadiansPerSecond);
    acceleration = filter.getLastAcceleration();
    acceleration = (acceleration.times(Constants.DriveConstants.ACCELERATION_FILTER_FACTOR)).plus(lastAcceleration.times(1.0-Constants.DriveConstants.ACCELERATION_FILTER_FACTOR));
    SmartDashboard.putNumberArray("Acceleration", Logger.chassisSpeedsToArray(acceleration));
    lastAcceleration = acceleration;
    return acceleration;
  }

  public SubsystemLog getPowerLog(){
    SubsystemLog flLog = modules[0].getModulePowerLog();
    SubsystemLog frLog = modules[1].getModulePowerLog();
    SubsystemLog blLog = modules[2].getModulePowerLog();
    SubsystemLog brLog = modules[3].getModulePowerLog();
    if(flLog == null){
      return null;
    }
    return new SubsystemLog("Drive", 
      new String[]{"Front Left Drive","Front Left Turn","Front Right Drive",
        "Front Right Turn","Back Left Drive","Back Left Turn","Back Right Drive","Back Right Turn"}, 
          new double[]{flLog.motorVolts()[0], flLog.motorVolts()[1], frLog.motorVolts()[0], frLog.motorVolts()[1], blLog.motorVolts()[0],
              blLog.motorVolts()[1], brLog.motorVolts()[0], brLog.motorVolts()[1]}, 
                new double[]{flLog.motorCurrents()[0], flLog.motorCurrents()[1], frLog.motorCurrents()[0], frLog.motorCurrents()[1], blLog.motorCurrents()[0],
                  blLog.motorCurrents()[1], brLog.motorCurrents()[0], brLog.motorCurrents()[1]});
  }

  public void lowerMaxAcceleration(){
    if((Constants.DriveConstants.DO_SHOOTING_ACCEL_LIMIT_IN_AUTO || !DriverStation.isAutonomous()) && inOurAllianceZone()){
      filter.setLimits(Constants.DriveConstants.DRIVE_MAX_VELOCITY, Constants.DriveConstants.DRIVE_MAX_ACCELERATION_SHOOTING, Constants.DriveConstants.DRIVE_MAX_ROTATION_VELOCITY_SHOOTING, Constants.DriveConstants.DRIVE_MAX_ROTATION_ACCELERATION);
    }
  }
  
  public void resetMaxAcceleration(){
    filter.setLimits(Constants.DriveConstants.DRIVE_MAX_VELOCITY, Constants.DriveConstants.DRIVE_MAX_ACCELERATION, Constants.DriveConstants.DRIVE_MAX_ROTATIONAL_VELOCITY, Constants.DriveConstants.DRIVE_MAX_ROTATION_ACCELERATION);
  }
}