package frc.WorBots.commands.pathPlannerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.spindexer.Spindexer;

/**
 * A command to stop firing
 */
public class StopShooting extends Command {
  Spindexer spin;

  /**
   * Stops firing by stopping running the spindexer
   * 
   * @param spin The spindexer to stop running
   */
  public StopShooting(Spindexer spin) {
    this.spin = spin;
  }

  @Override
  public void initialize() {
    spin.stopSpindexer();
  }
}
