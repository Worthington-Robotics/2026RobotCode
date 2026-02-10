package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.shooter.ShotCalculator;
import frc.WorBots.subsystems.shooter.ShotCalculator.ShootingParams;

/** A command to aim and prepare shots automatically */
public class ShooterAuto extends Command {
  private final Shooter shooter;
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
  }

  @Override
  public void execute(){
    ShootingParams params = shotCalculator.getParamsToHub(drive.getPose(), null); //TODO add a method to drive to get robot velocity
  }
  
}
