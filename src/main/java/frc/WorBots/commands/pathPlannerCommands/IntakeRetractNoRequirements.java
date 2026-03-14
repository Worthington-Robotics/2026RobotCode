package frc.WorBots.commands.pathPlannerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.intake.Intake;

/**
 * A command to retract the intake without requirements; for use with
 * pathplanner
 */
public class IntakeRetractNoRequirements extends Command {
  private Intake intake;

  /**
   * Retracts the intake without requirements; for use with pathplanner
   * 
   * @param intake The intake to retract
   */
  public IntakeRetractNoRequirements(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    intake.retract();
  }

  @Override
  public boolean isFinished() {
    return intake.atGoal();
  }
}
