package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.climber.Climber;
import frc.WorBots.subsystems.turret.Turret;

public class ClimberCommands {
  public Command resetClimber(Climber climber, Turret turret){
    return climber.runOnce(() -> {
      turret.unlockTurret();
      climber.setPosition(new Rotation2d());
    });
  }

  public Command readyClimber(Climber climber, Turret turret){
    return climber.runOnce(() -> {
      turret.runOnce(() -> {
        turret.lockTurret();
      });
      climber.setPosition(Constants.ClimberConstants.READY_CLIMBER_POSITION);
    });
  }

  public Command climb(Climber climber, Turret turret){
    return climber.runOnce(() -> {
      turret.runOnce(() -> {
        turret.lockTurret();
      });
      climber.setPosition(Constants.ClimberConstants.CLIMB_POSITION);
    });
  }
}
