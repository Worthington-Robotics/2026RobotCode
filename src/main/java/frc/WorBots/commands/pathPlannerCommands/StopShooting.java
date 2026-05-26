// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands.pathPlannerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.spindexer.Spindexer;

/**
 * A command to stop firing
 */
public class StopShooting extends Command {
  Spindexer spin;

  /**
   * Stops firing by stopping running the spindexer
   * 
   * @param spin The spindexer to stop running
   */
  public StopShooting(Spindexer spin) {
    this.spin = spin;
  }

  @Override
  public void initialize() {
    spin.stopSpindexer();
  }
}
