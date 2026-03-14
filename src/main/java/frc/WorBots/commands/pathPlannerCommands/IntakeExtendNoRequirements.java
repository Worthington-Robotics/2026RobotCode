package frc.WorBots.commands.pathPlannerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.intake.Intake;

/**
 * A command to extend the intake without requirements; for use with pathplanner
 */
public class IntakeExtendNoRequirements extends Command {
  private Intake intake;

  /**
   * Extends the intake without requirements; for use with pathplanner
   * 
   * @param intake The intake to extend
   */
  public IntakeExtendNoRequirements(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    intake.extend();
  }

  @Override
  public boolean isFinished() {
    return intake.atGoal();
  }
}
