package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.climber.Climber;

public class ClimberTestCommands {

  public Command climb(Climber climber){
    return climber.run(() -> {
      climber.setPosition(new Rotation2d(Math.PI));
    });
  }

  public Command voltClimb(Climber climber, double volts){
    return climber.startEnd(() -> {
      climber.setVolts(volts);
    }, () -> {
      climber.setVolts(0);
    });
  }

}
