// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.drive.Drive;

/** A class containing simple commands to interact with the drive subsystem */
public class DriveCommands {
  /**
   * Resets the heading of the drive; used for reseting forward for driver
   * controls
   * 
   * @param drive The drive to affect
   */
  public Command resetHeading(Drive drive) {
    return drive.runOnce(() -> {
      drive.resetYaw();
    });
  }
}
