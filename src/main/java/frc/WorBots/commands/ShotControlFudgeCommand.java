package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.superstructure.Superstructure;

/** Adjusts the shooter's flywheel fudge factor */
public class ShotControlFudgeCommand extends Command {

  private Superstructure shooter;
  private double fudgeFactor;

  /**
   * A command that runs once and adjusts the flywheel fudge factor.
   * 
   * @param shooter     The shooter for which to adjest the flywheel fudge factor.
   * @param fudgeFactor The amount to adjust the flywheel fudge factor by.
   */
  public ShotControlFudgeCommand(Superstructure shooter, double fudgeFactor) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.fudgeFactor = fudgeFactor;
  }

  @Override
  public void execute() {
    shooter.shooter.modFlywheelFudgeFactor(fudgeFactor);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
