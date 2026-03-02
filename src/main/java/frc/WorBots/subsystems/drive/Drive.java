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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.GyroIO.GyroIOInputs;
import frc.WorBots.util.OdometryThread;
import frc.WorBots.util.control.DriveFilter;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.math.GeomUtil;
import frc.WorBots.util.math.PoseEstimator;
import frc.WorBots.util.math.PoseEstimator.TimestampedVisionUpdate;

public class Drive extends SubsystemBase{
  private final Module[] modules = new Module[4];
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroIOInputs = new GyroIOInputs();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private DriveFilter filter = new DriveFilter(Constants.DriveConstants.DRIVE_MAX_VELOCITY, 
    Constants.DriveConstants.DRIVE_MAX_ACCELERATION, Constants.DriveConstants.DRIVE_MAX_ROTATIONAL_VELOCITY, 
    Constants.DriveConstants.DRIVE_MAX_ROTATION_ACCELERATION);

  private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();

  private ChassisSpeeds measuredSpeeds;

  private Rotation2d lastGyroYaw = new Rotation2d();

  private double[] lastModulePositionMeters = new double[4];

  private StopMode stopMode = StopMode.None;

  private PoseEstimator poseEstimator = new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));

  private final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private static final String TABLE_NAME = "Drive";
  private final NetworkTable driveTable = instance.getTable(TABLE_NAME);

  private final StructPublisher<ChassisSpeeds> speedSetpointPublisher = driveTable.getStructTopic("Speed Setpoint", ChassisSpeeds.struct).publish();
  private final StructPublisher<ChassisSpeeds> goalSetpointPublisher = driveTable.getStructTopic("Goal Speed Setpoint", ChassisSpeeds.struct).publish();
  private final StructPublisher<ChassisSpeeds> measuredSpeedPublisher = driveTable.getStructTopic("Measured Speed", ChassisSpeeds.struct).publish();
  private final DoubleArrayPublisher moduleSetpointPublisher = driveTable.getDoubleArrayTopic("Module Setpoints").publish();
  private final DoubleArrayPublisher moduleOptimizedPublisher = driveTable.getDoubleArrayTopic("Module Optimal").publish();
  private final DoubleArrayPublisher moduleMeasuredPublisher = driveTable.getDoubleArrayTopic("Module Measured").publish();
  private final StructPublisher<Pose2d> posePublisher = driveTable.getStructTopic("Pose", Pose2d.struct).publish();
  private final DoublePublisher gyroPublisher = driveTable.getDoubleTopic("Gyro Yaw").publish();
  private final DoublePublisher gyroVelocityPublisher = driveTable.getDoubleTopic("Gyro Velocity").publish();
  private final StringPublisher stopModePublisher = driveTable.getStringTopic("Stop Mode").publish();
  private final BooleanPublisher measuredStopPublisher = driveTable.getBooleanTopic("Measured Stop").publish();

  
  /**
   * Constructor for the Drive Subsytem
   * 
   * @param gyro The interface for a gyro that tracks the drivetrains orientation
   * @param flModule The interface of the drivetrain's front left module
   * @param frModule The interface of the drivetrain's front right module
   * @param blModule The interface of the drivetrain's back left module
   * @param brModule The interface of the drivetrain's back right module
   */

  public Drive(GyroIO gyro, ModuleIO flModule, ModuleIO frModule, ModuleIO blModule, ModuleIO brModule){
      gyroIO = gyro;
      modules[0] = new Module(flModule, 0);
      modules[1] = new Module(frModule, 1);
      modules[2] = new Module(blModule, 2);
      modules[3] = new Module(brModule, 3);
      StatusPage.reportStatus(StatusPage.DRIVE_SUBSYSTEM, true);
  }

  public void periodic(){
    //updates gyro data
    gyroIO.updateInputs(gyroIOInputs);

    //runs the modules
    for(Module a : modules){
      a.periodic();
    }

    //passes new information to the odometry and pose estimator
    updateOdometry();

    //publishes information to the drive table
    speedSetpointPublisher.set(setpointSpeeds);
    gyroPublisher.set(gyroIOInputs.yawPositionRad);
    gyroVelocityPublisher.set(gyroIOInputs.yawVelocityRadPerSec);
    stopModePublisher.set(stopMode.toString());
    measuredStopPublisher.set(isStopped());

    //Reports to Status page
    StatusPage.reportStatus(StatusPage.GYROSCOPE, gyroIOInputs.connected);

    //makes the robot move
    drive();
  }

  private void drive(){
    SwerveModuleState[] setpointStates;
    boolean forceModules = false;

    final ChassisSpeeds filteredFieldRelative = filter.calculate();
    setpointSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(filteredFieldRelative, getYaw());

    //if we're disabled the modules are all stoped, the goal speed is set to 0, and the filter is reset
    if(DriverStation.isDisabled()){
      for(Module a : modules){
        a.stop();
      }
      stop();
      filter.reset();
    } else{
      if(isStopped()){
          //TODO when polishing maybe change this so that the robot doesn't hard stop avery time
          setpointStates = setStop();
          forceModules = true;
      } else {
        setpointStates = kinematics.toSwerveModuleStates(setpointSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.DriveConstants.DRIVE_MAX_VELOCITY);
      }

      moduleSetpointPublisher.set(Logger.statesToArray(setpointStates));

      SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
      for(int i = 0; i < 4; i++){
        optimizedStates[i] = modules[i].optimizeState(setpointStates[i]);
      }

      moduleOptimizedPublisher.set(Logger.statesToArray(optimizedStates));

      for(int i = 0; i < 4; i++){
        modules[i].runState(optimizedStates[i], forceModules);
      }
    }

  }

  private Translation2d[] getModuleTranslations(){
    return Constants.DriveConstants.DRIVE_MODULE_OFFSETS;
  }

  /**
   * Tells the robot to move at a velocity of 0, does not directly use stop modes
   */
  public void stop(){
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Moves all modules to form a x causing the robot to hardstop in place
   */
  public SwerveModuleState[] setStop(){
    SwerveModuleState[] setpointStates = new SwerveModuleState[4];
    
    for(int i = 0; i < 4; i++){
      setpointStates[i] = new SwerveModuleState(0.0, modules[i].getAngle() );
    }
    return setpointStates;
  }

  /**
   * Updates the drive filter with the new field relative speed the robot should be driving at
   * 
   * @param speeds the robot relative speed being requested of the robot
   */
  public void runVelocity(ChassisSpeeds speeds){
    ChassisSpeeds ajusted = GeomUtil.driftCorrectChassisSpeeds(speeds, Constants.DriveConstants.DRIVE_DRIFT_RATE);
    goalSetpointPublisher.set(ajusted);

    //Calculates a field relative velocity as if we're on blue, then flips it to red if nessesary
    ChassisSpeeds fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getYaw());
    //ChassisSpeeds allianceRel = AllianceFlipUtil.flipSpeeds(fieldRel);
    filter.setGoal(fieldRel);
  }

  /**
   * Checks our linear and rotational velocity to see if the robot is physicially stopped
   */
  public boolean isStopped(){
    double magnitude = Math.hypot(setpointSpeeds.vxMetersPerSecond, setpointSpeeds.vyMetersPerSecond);
    return magnitude < Constants.DriveConstants.DRIVE_STOP_XY_THRESHOLD && 
      Math.abs(setpointSpeeds.omegaRadiansPerSecond) < Constants.DriveConstants.DRIVE_THETA_THRESHOLD;
  }

  /**
   * Returns the robots rotation acording to the drivetrain's gyroscope
   */
  public Rotation2d getYaw(){
    return new Rotation2d(gyroIOInputs.yawPositionRad);
  }

  /**
   * Returns the robots rotational velocity acording to the drivetrain's gyroscope
   */
  public Rotation2d getYawVelocity(){
    return new Rotation2d(gyroIOInputs.yawVelocityRadPerSec);
  }

  /**
   * Returns the robot relative velocity, mostly for pathPlanner
   */
  public ChassisSpeeds getRobotRelativeSpeeds(){
    return ChassisSpeeds.fromFieldRelativeSpeeds(filter.calculate(), getYaw());
  }

  /**
   * Passes new drive information to the Odometry threat and pose estimator
   */
  public void updateOdometry(){
    final double startTime = Timer.getFPGATimestamp();
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];

    for(int i=0; i<4; i++){
      measuredStates[i] = modules[i].getState();
    }
    moduleMeasuredPublisher.set(Logger.statesToArray(measuredStates));

    measuredSpeeds = kinematics.toChassisSpeeds(measuredStates);

    OdometryThread.odometryLock.lock();
    ArrayList<Double> timestamps = new ArrayList<>(OdometryThread.timestampQueue.size());
    timestamps.clear();

    while(OdometryThread.timestampQueue.size() > 0){
      final double timestamp = OdometryThread.timestampQueue.poll();
      timestamps.add(timestamp);
    }

    OdometryThread.odometryLock.unlock();

    int minSize = timestamps.size();
    for(Module module : modules){
      minSize = Math.min(minSize, module.getDrivePositionUpdates().size());
      minSize = Math.min(minSize, module.getTurnPositionUpdates().size());
    }
    if(gyroIOInputs.connected){
      minSize = Math.min(minSize, gyroIOInputs.yawPositionUpdates.size());
    } else {
      minSize = 0;
    }

    int modulus = 1;

    if(minSize > 0){
      if(minSize > 15){
        modulus = 2;

      } else if(minSize > 30) {
        modulus = 3;
      }
    }
    int update = 0;
    for(update = 0; update < minSize / modulus; update++){
      update = update * modulus;

      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];

      for(int i = 0; i < 4; i++){
        final Module module = modules[i];
        final double distance = module.getDrivePositionUpdates().get(update);
        final Rotation2d angle = new Rotation2d(module.getTurnPositionUpdates().get(update));

        wheelDeltas[i] = new SwerveModulePosition((distance - lastModulePositionMeters[i]), angle);
        lastModulePositionMeters[i] = distance;
      }

      final Twist2d twist = kinematics.toTwist2d(wheelDeltas);

      final Rotation2d gyroYaw = new Rotation2d(gyroIOInputs.yawPositionUpdates.get(update));
      if(gyroIOInputs.connected) {
        final double dtheta = gyroYaw.minus(lastGyroYaw).getRadians();

        twist.dtheta = dtheta;
      }

      lastGyroYaw = gyroYaw;

      poseEstimator.addDriveDataNoUpdate(timestamps.get(update), twist);
      posePublisher.set(getPose());

      gyroIO.setExpectedYawVelocity(measuredSpeeds.omegaRadiansPerSecond);
    }
    measuredSpeedPublisher.set(measuredSpeeds);
    poseEstimator.update();

    final double endTime = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Odom Time", endTime - startTime);
  }

  /**
   * sets the robots pose and rotation
   * @return
   */
  public void resetPose(Pose2d pose){
    poseEstimator.resetPose(pose);
  }

  /**
   * Returns the robots rotation according to the pose estimator
   */
  public Rotation2d getRotation(){
    return poseEstimator.getLatestPose().getRotation();
  }
  
  /**
   * Returns the robots position according to the pose estimator
   */
  public Pose2d getPose(){
    return poseEstimator.getLatestPose();
  }

  /**
   * Returns the robot's measured speeds
   */
  public ChassisSpeeds getMeasuredSpeeds(){
    return measuredSpeeds;
  }

  public void addVisionUpdate(List<TimestampedVisionUpdate> update){
    poseEstimator.addVisionData(update);
  }
}