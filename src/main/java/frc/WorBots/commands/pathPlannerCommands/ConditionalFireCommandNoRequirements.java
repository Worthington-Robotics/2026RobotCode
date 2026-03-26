package frc.WorBots.commands.pathPlannerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.spindexer.Spindexer;
import frc.WorBots.util.FireController;

/** A command to shot if the fire control system believes we can shoot. Built without requirements for use with pathplanner. */
public class ConditionalFireCommandNoRequirements extends Command {
  public Spindexer spin;
  public Drive drive;
  double voltage;

  /**
   * Shoots when the fire control system believes we can shoot. Also lowers drive
   * max velocity while firing. Built without requirements for use with pathplanner.
   * 
   * @param drive   The drive subsystem; used for getting robot pose
   * @param spin    The spindexer to run to feed to shooter
   * @param voltage The voltage to run the spindexer at
   */
  public ConditionalFireCommandNoRequirements(Drive drive, Spindexer spin, double voltage) {
    this.spin = spin;
    this.drive = drive;
    this.voltage = voltage;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Lowers drive max speed and if we are ready to fire fires
    drive.setDriveMaxSpeed(Constants.DriveConstants.DRIVE_MAX_VELOCITY / 2);
    if (FireController.getInstance().readyToFire()) {
      spin.setVelocity(Constants.SpindexerConstants.SPINDEXER_VELOCITY, Constants.SpindexerConstants.KICKER_VELOCITY);
    }
  }

  @Override
  public void end(boolean interupted) {
    spin.stopSpindexer();
    drive.setDriveMaxSpeed(Constants.DriveConstants.DRIVE_MAX_VELOCITY);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
