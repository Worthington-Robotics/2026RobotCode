package frc.WorBots.subsystems.drive;

import java.util.ArrayList;

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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.GyroIO.GyroIOInputs;
import frc.WorBots.util.OdometryThread;
import frc.WorBots.util.control.DriveFilter;
import frc.WorBots.util.debug.Logger;
import frc.WorBots.util.math.GeomUtil;
import frc.WorBots.util.math.PoseEstimator;

public class Drive extends SubsystemBase{
  private final Module[] modules = new Module[4];
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroIOInputs = new GyroIOInputs();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private DriveFilter filter = new DriveFilter(Constants.DRIVE_MAX_VELOCITY, Constants.DRIVE_MAX_ACCELERATION, Constants.DRIVE_MAX_ROTATIONAL_VELOCITY, Constants.DRIVE_MAX_ROTATION_ACCELERATION);

  private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();

  private Twist2d fieldVelocity = new Twist2d();

  private ChassisSpeeds measurdSpeeds;

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

      //TODO remove when we acually have autos to set a real start pose
      poseEstimator.resetPose(new Pose2d(3, 3, new Rotation2d()));
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
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, Constants.DRIVE_MAX_VELOCITY);
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
    return new Translation2d[] {
      new Translation2d(Constants.ROBOT_WHEELBASE / 2, Constants.ROBOT_WHEELBASE / 2),
      new Translation2d(Constants.ROBOT_WHEELBASE / 2, -Constants.ROBOT_WHEELBASE / 2),
      new Translation2d(-Constants.ROBOT_WHEELBASE / 2, Constants.ROBOT_WHEELBASE / 2),
      new Translation2d(-Constants.ROBOT_WHEELBASE / 2, -Constants.ROBOT_WHEELBASE / 2)
    };
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
      setpointStates[i] = new SwerveModuleState(0.0, new Rotation2d(StopMode.BlockAll.moduleAngles[i]));
    }
    return setpointStates;
  }

  /**
   * Updates the drive filter with the new field relative speed the robot should be driving at
   * 
   * @param speeds the robot relative speed being requested of the robot
   */
  public void runVelocity(ChassisSpeeds speeds){
    ChassisSpeeds ajusted = GeomUtil.driftCorrectChassisSpeeds(speeds, Constants.DRIVE_DRIFT_RATE);
    goalSetpointPublisher.set(ajusted);

    ChassisSpeeds fieldRel = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getYaw());
    filter.setGoal(fieldRel);
  }

  /**
   * Checks our linear and rotational velocity to see if the robot is physicially stopped
   */
  public boolean isStopped(){
    double magnitude = Math.hypot(setpointSpeeds.vxMetersPerSecond, setpointSpeeds.vyMetersPerSecond);
    return magnitude < Constants.DRIVE_STOP_XY_THRESHOLD && Math.abs(setpointSpeeds.omegaRadiansPerSecond) < Constants.DRIVE_THETA_THRESHOLD;
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
   * Passes new drive information to the Odometry threat and pose estimator
   */
  public void updateOdometry(){
    final double startTime = Timer.getFPGATimestamp();
    SwerveModuleState[] meauredStates = new SwerveModuleState[4];

    for(int i=0; i<4; i++){
      meauredStates[i] = modules[i].getState();
    }
    moduleMeasuredPublisher.set(Logger.statesToArray(meauredStates));

    measurdSpeeds = kinematics.toChassisSpeeds(meauredStates);
    final Translation2d linearFieldVelocity =
      new Translation2d(measurdSpeeds.vxMetersPerSecond, measurdSpeeds.vyMetersPerSecond)
      .rotateBy(getRotation());
    
    fieldVelocity = new Twist2d(
      linearFieldVelocity.getX(),
      linearFieldVelocity.getY(),
      gyroIOInputs.connected
        ? gyroIOInputs.yawVelocityRadPerSec : measurdSpeeds.omegaRadiansPerSecond);

    OdometryThread.odometryLock.lock();
    ArrayList<Double> timestamps = new ArrayList<>(OdometryThread.timestampQueue.size());
    timestamps.clear();

    while(OdometryThread.timestampQueue.size() > 0){
      final double timestamp = OdometryThread.timestampQueue.poll();
      timestamps.add(timestamp);
    }
    SmartDashboard.putNumberArray("timestamps", timestamps.toArray(new Double[0]));

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
    SmartDashboard.putNumber("minSize", minSize);
    int update = 0;
    double temp[] = new double[3];
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

        SmartDashboard.putNumber("Odometry dTheta Error", dtheta - twist.dtheta);
        twist.dtheta = dtheta;
      }

      lastGyroYaw = gyroYaw;

      poseEstimator.addDriveDataNoUpdate(timestamps.get(update), twist);
      temp[0] = twist.dx;
      temp[1] = twist.dy;
      temp[2] = twist.dtheta;
      posePublisher.set(getPose());

      gyroIO.setExpectedYawVelocity(measurdSpeeds.omegaRadiansPerSecond);
    }

    SmartDashboard.putNumber("odometry update", update);
    SmartDashboard.putNumberArray("odometry update twist", temp);

    poseEstimator.update();

    final double endTime = Timer.getFPGATimestamp();
    SmartDashboard.putNumber("Odom Time", endTime - startTime);
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
   * @return wether the robot is in its alliance zone, returns false if it doesn't have a alliance
   */
  public Boolean isNear(){
    Pose2d currentPose = getPose();
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Blue){
        return currentPose.getX() < FieldConstants.BLUE_ZONE_LINE_X_CORD;
      } else{
        return currentPose.getX() > FieldConstants.RED_ZONE_LINE_X_CORD;
      }
    }

    return false;
  }
}