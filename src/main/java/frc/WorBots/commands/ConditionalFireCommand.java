package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.spindexer.Spindexer;
import frc.WorBots.util.FireController;

/** A command to shot if the fire control system believes we can shoot */
public class ConditionalFireCommand extends Command {
  public Spindexer spin;
  public Drive drive;
  double voltage;

  /**
   * Shoots when the fire control system believes we can shoot. Also lowers drive
   * max velocity while firing.
   * 
   * @param drive   The drive subsystem; used for getting robot pose
   * @param spin    The spindexer to run to feed to shooter
   * @param voltage The voltage to run the spindexer at
   */
  public ConditionalFireCommand(Drive drive, Spindexer spin,double voltage) {
    this.spin = spin;
    this.drive = drive;
    this.voltage = voltage;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (FireController.getInstance().readyToFire()) {
      spin.setVelocity(Constants.SpindexerConstants.SPINDEXER_VELOCITY, Constants.SpindexerConstants.KICKER_VELOCITY);
    } else {
      spin.setVelocity(0.0, Constants.SpindexerConstants.KICKER_VELOCITY);
    }
  }

  @Override
  public void end(boolean interupted) {
    spin.stopSpindexer();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
