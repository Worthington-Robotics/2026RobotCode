package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.climber.Climber;
import frc.WorBots.subsystems.turret.Turret;

public class ClimberCommands {
  public Command resetClimber(Climber climber, Turret turret){
    return Commands.sequence(
      climber.runOnce(() -> {
        climber.setPosition(new Rotation2d());
      }),
      Commands.waitUntil(() -> climber.atGoal()),
      turret.runOnce(()-> {
        turret.unlockTurret();
      }));
  }


  public Command readyClimber(Climber climber, Turret turret){
    return Commands.sequence(
      turret.runOnce(()->{turret.lockTurret();}),
      Commands.waitUntil(() -> turret.atGoal()),
      climber.runOnce(()->{
        climber.setPosition(Constants.ClimberConstants.READY_CLIMBER_POSITION);
      }));
    }

  public Command climb(Climber climber, Turret turret){
    return Commands.sequence(
      turret.runOnce(()->{turret.lockTurret();}),
      Commands.waitUntil(() -> turret.atGoal()),
      climber.runOnce(()->{
        climber.setPosition(Constants.ClimberConstants.CLIMB_POSITION);
      }));
    }
}

