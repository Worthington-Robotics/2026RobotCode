package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.superstructure.Superstructure;

/** A command to change the hood fudge factor */
public class HoodControlFudgeCommand extends Command {

  private Superstructure superstructure;
  private double fudgeFactor;
  private Supplier<Boolean> doRunSupplier;

  /**
   * Changes the hood fudge factor by a specified amount.
   * 
   * @param superstructure The superstructure containing the hood to adjust the
   *                       fudge factor for
   * @param fudgeFactor    The amount to adjust the fudge factor by
   * @param doRunSupplier  If true the command will run
   */
  public HoodControlFudgeCommand(Superstructure superstructure, double fudgeFactor, Supplier<Boolean> doRunSupplier) {
    addRequirements(superstructure);
    this.superstructure = superstructure;
    this.fudgeFactor = fudgeFactor;
    this.doRunSupplier = doRunSupplier;
  }

  @Override
  public void execute() {
    if(doRunSupplier.get()){
      superstructure.shooter.modHoodFudgeFactor(fudgeFactor);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
