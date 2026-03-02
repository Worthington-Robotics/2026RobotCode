package frc.WorBots.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.spindexer.Spindexer;
import frc.WorBots.subsystems.turret.Turret;

public class ShooterCommands {
  /**
   * Resets the position of the hood.
   * @param shooter
   * @return Command to reset hood position
   */
  public Command resetHoodPosition(Shooter shooter){
    return shooter.runOnce(()->{
      shooter.resetHoodPosition();
    });
  }

  public Command setHoodPose(Shooter shooter, double hoodPose){
    return shooter.runOnce(() -> {
      shooter.setHoodPose(hoodPose);
    });
  }

  public Command setFlyWheel(Shooter shooter, double speed){
    return shooter.runOnce(() -> {
      shooter.setFlywheelSpeed(speed);
    });
  }

  public Command feedShooter(Spindexer spin, Shooter shooter, Turret turret){
    return spin.runEnd(() -> {
      if(turret.atGoal() && shooter.readyToShoot()){
        spin.runSpindexer();
      } else {
        spin.stopSpindexer();
      }
    }, () -> {
      spin.stopSpindexer();
    });
  }

    public Command forceFeedShooter(Spindexer spin){
    return spin.runEnd(() -> {
      spin.runSpindexer();
    }, () -> {
      spin.stopSpindexer();
    });
  }

  
  
  //TODO: Create more commands. 



}
