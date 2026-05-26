// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.superstructure.Superstructure;

/** Adjusts the shooter's flywheel fudge factor */
public class ShotControlFudgeCommand extends Command {

  private Superstructure shooter;
  private double fudgeFactor;
  private Supplier<Boolean> doRunSupplier;

  /**
   * A command that runs once and adjusts the flywheel fudge factor.
   * 
   * @param shooter       The shooter for which to adjest the flywheel fudge
   *                      factor.
   * @param fudgeFactor   The amount to adjust the flywheel fudge factor by.
   * @param doRunSupplier If true the command will run
   */
  public ShotControlFudgeCommand(Superstructure shooter, double fudgeFactor, Supplier<Boolean> doRunSupplier) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.fudgeFactor = fudgeFactor;
    this.doRunSupplier = doRunSupplier;
  }

  @Override
  public void execute() {
    if (doRunSupplier.get()) {
      shooter.shooter.modFlywheelFudgeFactor(fudgeFactor);
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
