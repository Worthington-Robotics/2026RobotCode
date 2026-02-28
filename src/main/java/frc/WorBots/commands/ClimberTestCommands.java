package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public Command climbWIthStick(Climber climber, Supplier<Double> volts){
    return climber.startEnd(() -> {
      climber.setVolts(volts.get());
    }, () -> {
      climber.setVolts(0);
    });
  }

}
