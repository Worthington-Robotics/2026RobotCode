// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

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
    return true;
  }
}
