// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.lights.Lights.LightModes;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.Superstructure.SuperstructureControlMode;

/** A command to aim and prepare shots automatically */
public class StartAutoAim extends Command {
  private final Superstructure superstructure;

  /**
   * A command to aim and prepare shots automatically
   * 
   * @param shooter The shooter to use
   * @param turret  The turret to use
   * @param drive   The robot's drivetrain, used for fetching pose.
   */
  public StartAutoAim(Superstructure superstructure) {
    this.superstructure = superstructure;
  }

  @Override
  public void initialize() {
    superstructure.setControlMode(SuperstructureControlMode.AutomaticShot);
    ;
    Lights.getInstance().setMode(LightModes.TurretDisplay);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
