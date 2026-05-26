// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands.pathPlannerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.subsystems.intake.Intake;

public class PathplannerIntakeCommands {
  /**
   * A command to start running the intake without requirements
   * 
   * @param intake The intake to start running
   */
  public Command startIntakeAuto(Intake intake) {
    return Commands.runOnce(() -> {
      intake.setVoltsIntake(8);
    });
  }

  /**
   * A command to stop running the intake without requirement
   * 
   * @param intake The intake to stop running
   */
  public Command stopIntakeAuto(Intake intake) {
    return Commands.runOnce(() -> {
      intake.setVoltsIntake(0);
    });
  }
}
