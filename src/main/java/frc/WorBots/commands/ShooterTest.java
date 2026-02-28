package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.shooter.ShotCalculator;
import frc.WorBots.subsystems.shooter.ShotCalculator.ShootingParams;
import frc.WorBots.subsystems.turret.Turret;
import frc.WorBots.util.math.AllianceFlipUtil;

/** A command to aim and prepare shots automatically */
public class ShooterTest extends Command {
  private final Shooter shooter;
  private final Turret turret;
  private final Drive drive;
  private ShotCalculator shotCalculator = ShotCalculator.getInstance();
  Field2d field = new Field2d();

  private Pose2d pose;

  /** A command to aim and prepare shots automatically
   * @param shooter The shooter to use
   * @param drive The robot's drivetrain, used for fetching pose.
   */
  public ShooterTest(Shooter shooter, Drive drive, Turret turret){
    addRequirements(shooter, turret);
    this.shooter = shooter;
    this.drive = drive;
    this.turret = turret;
    SmartDashboard.putData(field);
  }

  @Override
  public void execute(){
    pose = drive.getPose();
    ShootingParams params;
    
    if(AllianceFlipUtil.apply(pose).getX()< FieldConstants.hubPosition.getX()){
      params = shotCalculator.getParamsToHub(pose, drive.getMeasuredSpeeds()); 
    } else {
      params = shotCalculator.getPassParams(pose, drive.getMeasuredSpeeds()); 
      SmartDashboard.putBoolean("Pass valid", params.isValid());
    }
    if(params.isValid()){
      turret.setPositionAndVelocity(params, pose);
      SmartDashboard.putNumber("Turret Velocity", params.turretVelocity());
      SmartDashboard.putNumber("Turret Angle", params.turretAngle().getDegrees());
      shooter.setShooterParams(params);
      //Calculate the final position of the shot
      double time = 2*params.flywheelspeed() * Math.sin(params.hoodAngle()) / 9.8;
      double velocity = params.flywheelspeed() * Math.cos(params.hoodAngle());
      double xVelocity = velocity * Math.cos(params.turretAngle().getRadians())+drive.getMeasuredSpeeds().vxMetersPerSecond;
      double yVelocity = velocity * Math.sin(params.turretAngle().getRadians())+drive.getMeasuredSpeeds().vyMetersPerSecond;
      field.setRobotPose(new Pose2d(pose.getX()+xVelocity*time, pose.getY()+yVelocity*time, new Rotation2d()));
    } else {
      //field.setRobotPose(new Pose2d());
      field.getObject("Predicted Impact Site").setPose(AllianceFlipUtil.apply( new Pose2d(FieldConstants.passTarget.getX(), FieldConstants.passTarget.getY()+(.5* FieldConstants.fieldLength), new Rotation2d())));
    }
    SmartDashboard.putNumberArray("TurretPose", new double[] {pose.getX(),pose.getY(),turret.getPosition()+ pose.getRotation().getRadians()});

    }
}
