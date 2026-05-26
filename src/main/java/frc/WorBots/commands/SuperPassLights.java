package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.lights.Lights;

/**
 * A command to play rainbow lights; for use during super passing
 */
public class SuperPassLights extends Command {

  /**
   * A command to play rainbow lights; for use during super passing
   */
  public SuperPassLights() {
  }

  @Override
  public void initialize() {
    Lights.getInstance().superStar(true);
  }

  @Override
  public void end(boolean interupted) {
    Lights.getInstance().superStar(false);
  }
}
