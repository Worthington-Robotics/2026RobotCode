package frc.WorBots.subsystems.drive;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.GyroIO;
import frc.WorBots.subsystems.drive.Module;
import frc.WorBots.subsystems.drive.ModuleIO;
import frc.WorBots.subsystems.drive.GyroIO.GyroIOInputs;
import frc.WorBots.util.control.DriveFilter;

public class Drive extends SubsystemBase{
  private final Module[] modules = new Module[4];
  private final GyroIO gyroIO;
  private final GyroIOInputs gyroIOInputs = new GyroIOInputs();

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private DriveFilter filter = new DriveFilter(Constants.DRIVE_MAX_VELOCITY, Constants.DRIVE_MAX_ACCELERATION, Constants.DRIVE_MAX_ROTATIONAL_VELOCITY, Constants.DRIVE_MAX_ROTATION_ACCELERATION);

  private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();

  private Twist2d fieldVelocity = new Twist2d();

  private ChassisSpeeds measurSpeeds;

  private Rotation2d lastGyroYaw = new Rotation2d();

  private double[] lastModulePositionMeters = new double[4];

  private StopMode stopMode = StopMode.None;

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

  

  public Drive(GyroIO gyro, ModuleIO flModule, ModuleIO frModule, ModuleIO blModule, ModuleIO brModule){
      gyroIO = gyro;
      modules[0] = new Module(flModule, 0);
      modules[1] = new Module(frModule, 1);
      modules[2] = new Module(blModule, 2);
      modules[3] = new Module(brModule, 3);

      
  }

  public void periodic(){
    gyroIO.updateInputs(gyroIOInputs);

    for(Module a : modules){
      a.periodic();
    }

    //Update Odometry Here

    speedSetpointPublisher.set(setpointSpeeds);
    gyroPublisher.set(gyroIOInputs.yawPositionRad);
    gyroVelocityPublisher.set(gyroIOInputs.yawVelocityRadPerSec);
    stopModePublisher.set(stopMode.toString());
    measuredStopPublisher.set(isStopped());

    drive();
  }

  private void drive(){
    final ChassisSpeeds filteredFieldRelative = filter.calculate();
    setpointSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(filteredFieldRelative, getYaw());

    if(DriverStation.isDisabled()){
      for(Module a : modules){
        a.stop();
      }
      stop();
      filter.reset();
    } else{
      //TODO rest of drive code here
    }

  }

  private Translation2d[] getModuleTranslations(){
    return new Translation2d[] {
      new Translation2d(Constants.ROBOT_WHEELBASE, Constants.ROBOT_WHEELBASE),
      new Translation2d(Constants.ROBOT_WHEELBASE, -Constants.ROBOT_WHEELBASE),
      new Translation2d(-Constants.ROBOT_WHEELBASE, Constants.ROBOT_WHEELBASE),
      new Translation2d(-Constants.ROBOT_WHEELBASE, -Constants.ROBOT_WHEELBASE)
    };
  }

  public void stop(){
    //TODO make this
  }

  public boolean isStopped(){
    //TODO Make this as well, should use actual math not just that we said to stop
    return false;
  }

  public Rotation2d getYaw(){
    return new Rotation2d(gyroIOInputs.yawPositionRad);
  }
}