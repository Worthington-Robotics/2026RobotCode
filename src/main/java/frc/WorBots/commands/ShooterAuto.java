package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.shooter.ShotCalculator;
import frc.WorBots.subsystems.shooter.ShotCalculator.ShootingParams;
import frc.WorBots.subsystems.turret.Turret;
import frc.WorBots.util.math.AllianceFlipUtil;

/** A command to aim and prepare shots automatically */
public class ShooterAuto extends Command {
  private final Shooter shooter;
  private final Turret turret;
  private final Drive drive;
  private ShotCalculator shotCalculator = new ShotCalculator();

  private Pose2d pose;

  /** A command to aim and prepare shots automatically
   * @param shooter The shooter to use
   * @param turret The turret to use
   * @param drive The robot's drivetrain, used for fetching pose.
   */
  public ShooterAuto(Shooter shooter, Turret turret, Drive drive){
    addRequirements(shooter);
    addRequirements(turret);
    this.turret = turret;
    this.shooter = shooter;
    this.drive = drive;
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
      shooter.setShooterParams(params);
      turret.setPosition(params, pose);
    }
    }
}
