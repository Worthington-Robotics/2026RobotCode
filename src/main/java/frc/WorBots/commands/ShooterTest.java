package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.shooter.ShotCalculator;
import frc.WorBots.subsystems.shooter.ShotCalculator.ShootingParams;
import frc.WorBots.util.math.AllianceFlipUtil;

/** A command to aim and prepare shots automatically */
public class ShooterTest extends Command {
  private final Shooter shooter;
  //TODO add turret to this code
  private final Drive drive;
  private ShotCalculator shotCalculator = new ShotCalculator();
  Field2d field = new Field2d();

  private Pose2d pose;

  /** A command to aim and prepare shots automatically
   * @param shooter The shooter to use
   * @param drive The robot's drivetrain, used for fetching pose.
   */
  public ShooterTest(Shooter shooter, Drive drive){
    addRequirements(shooter);
    this.shooter = shooter;
    this.drive = drive;
    //TODO add turret here
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
    }
    if(params.isValid()){
      //Calculate the final position of the shot
      double time = 2*params.flywheelspeed() * Math.sin(params.hoodAngle()) / 9.8;
      double velocity = params.flywheelspeed() * Math.cos(params.hoodAngle());
      double xVelocity = velocity * Math.cos(params.turretAngle().getRadians());
      double yVelocity = velocity * Math.sin(params.turretAngle().getRadians());
      field.setRobotPose(pose.exp(new Twist2d(xVelocity, yVelocity, 0)));
    }
    }
}
