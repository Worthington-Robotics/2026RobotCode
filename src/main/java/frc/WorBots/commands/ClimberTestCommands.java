package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.climber.Climber;
import frc.WorBots.util.debug.TunableDouble;

/** Contains commands used for testing the climber */
public class ClimberTestCommands {

  TunableDouble voltageDouble = new TunableDouble("Tuning", "Climber", "Voltage Mult");

  /**
   * Sets the climber position to a temporary climbing position
   * 
   * @param climber The climber to climb with
   */
  public Command climb(Climber climber) {
    return climber.run(() -> {
      climber.setPosition(new Rotation2d(Math.PI));
    });
  }

  /**
   * Runs the climber at a voltage
   * 
   * @param climber The climber to run
   * @param volts   The voltage to run the climber at
   */
  public Command voltClimb(Climber climber, double volts) {
    return climber.startEnd(() -> {
      climber.setVolts(volts);
    }, () -> {
      climber.setVolts(0);
    });
  }

  /**
   * Commands the climber with a joystick input
   * 
   * @param climber The climber to command
   * @param volts   A supplier providiing the stick value controlling the voltage
   *                to set the climber to
   * @return
   */
  public Command climbWIthStick(Climber climber, Supplier<Double> volts) {
    return climber.runOnce(() -> {
      climber.setVolts(volts.get() * 2);
    });
  }

}
