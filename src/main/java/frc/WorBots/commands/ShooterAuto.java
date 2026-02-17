package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.shooter.ShotCalculator;
import frc.WorBots.subsystems.shooter.ShotCalculator.ShootingParams;
import frc.WorBots.util.math.AllianceFlipUtil;

/** A command to aim and prepare shots automatically */
public class ShooterAuto extends Command {
  private final Shooter shooter;
  //TODO add turret to this code
  private final Drive drive;
  private ShotCalculator shotCalculator = new ShotCalculator();

  private Pose2d pose;

  /** A command to aim and prepare shots automatically
   * @param shooter The shooter to use
   * @param drive The robot's drivetrain, used for fetching pose.
   */
  public ShooterAuto(Shooter shooter, Drive drive){
    addRequirements(shooter);
    this.shooter = shooter;
    this.drive = drive;
    //TODO add turret here
  }

  @Override
  public void execute(){
    pose = drive.getPose();
    ShootingParams params;
    if(AllianceFlipUtil.apply(pose).getX()< FieldConstants.hubPosition.getX()){
      params = shotCalculator.getParamsToHub(pose, drive.getMeasuredSpeeds()); //TODO add a method to drive to get robot velocity
    } else {
      params = shotCalculator.getPassParams(pose, drive.getMeasuredSpeeds()); //TODO add a method to drive to get robot velocity
    }
    if(params.isValid()){
      shooter.setShooterParams(params);
      //TODO set turret
    }
    }
}
