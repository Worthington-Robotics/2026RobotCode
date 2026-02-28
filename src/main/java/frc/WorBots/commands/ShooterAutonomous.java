package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.FieldConstants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.shooter.ShotCalculator;
import frc.WorBots.subsystems.shooter.ShotCalculator.ShootingParams;
import frc.WorBots.subsystems.spindexer.Spindexer;
import frc.WorBots.subsystems.turret.Turret;
import frc.WorBots.util.math.AllianceFlipUtil;

/** A command to aim and prepare shots automatically */
public class ShooterAutonomous extends Command {
  private final Shooter shooter;
  private final Turret turret;
  private final Drive drive;
  private final Spindexer spin;
  private ShotCalculator shotCalculator = ShotCalculator.getInstance();

  private Pose2d pose;

  /** A command to aim and shoots automatically (specifically for use in autos)
   * @param shooter The shooter to use
   * @param turret The turret to use
   * @param drive The robot's drivetrain, used for fetching pose.
   * @param spin The spindexer to use
   */
  public ShooterAutonomous(Shooter shooter, Turret turret, Drive drive, Spindexer spin){
    addRequirements(shooter, turret, spin);
    this.turret = turret;
    this.shooter = shooter;
    this.drive = drive;
    this.spin = spin;
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
      turret.setPositionAndVelocity(params, pose);
      //TODO add shot is valid to status page
    }
    if(shooter.readyToShoot() && turret.atGoal() && params.isValid()){
      spin.runSpindexer();
    } else {
      spin.stopSpindexer();
    }
    }
}
