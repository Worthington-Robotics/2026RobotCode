// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.spindexer.Spindexer;

/** A command to run the spindexer */
public class RunSpindexer extends Command {
  private final Spindexer spin;
  private double voltage = 0;

  /**
   * A command to run the spindexer.
   * 
   * @param spin    The spindexer to run
   * @param voltage The voltage to run the spindexer at
   */
  public RunSpindexer(Spindexer spin, double voltage) {
    addRequirements(spin);
    this.spin = spin;
    this.voltage = voltage;
  }

  @Override
  public void initialize() {
    if (voltage < 0) {
      spin.runSpindexerVoltage(voltage);
    } else {
      spin.setVelocity(Constants.SpindexerConstants.SPINDEXER_VELOCITY, Constants.SpindexerConstants.KICKER_VELOCITY);
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interupted) {
    spin.stopSpindexer();
  }
}
