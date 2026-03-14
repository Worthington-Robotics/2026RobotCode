package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.spindexer.Spindexer;
import frc.WorBots.util.FireController;

/**
 * A command that does nothing and ends once the robot can fire
 */
public class WaitUntilReadyToFire extends Command {
  public Spindexer spin;
  boolean good;

  /**
   * A command that does nothing and ends once the robot can fire
   */
  public WaitUntilReadyToFire(Spindexer spin, double voltage) {
    addRequirements(spin);
    this.spin = spin;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interupted) {

  }

  @Override
  public boolean isFinished() {
    return FireController.getInstance().readyToFire();
  }

}
