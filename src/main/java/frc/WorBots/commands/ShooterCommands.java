package frc.WorBots.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.shooter.Shooter;

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

  
  
  //TODO: Create more commands. 



}
