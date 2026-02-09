// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.WorBots;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.WorBots.commands.DriveWithJoysticks;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.drive.GyroIOPigeon2;
import frc.WorBots.subsystems.drive.GyroIOSim;
import frc.WorBots.subsystems.drive.ModuleIOSim;
import frc.WorBots.subsystems.drive.ModuleIOTalon;
import frc.WorBots.util.control.DriveController;

public class RobotContainer {
  //Subsystems
  public final Drive drive;

  //Joysticks
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController operator = new CommandXboxController(1);

  //Drive Controller
  public static final DriveController driveController = new DriveController();

  public RobotContainer() {
    //setup Subsystems
    if(!Constants.getSim()){
      drive = new Drive(
        new GyroIOPigeon2(), 
        new ModuleIOTalon(0), 
        new ModuleIOTalon(1), 
        new ModuleIOTalon(2), 
        new ModuleIOTalon(3));
    } else {
      drive = new Drive(
        new GyroIOSim(), 
        new ModuleIOSim(0), 
        new ModuleIOSim(1), 
        new ModuleIOSim(2), 
        new ModuleIOSim(3));
    }

    AutoBuilder.configure(
      () -> drive.getPose(), //Get Pose Command
      pose -> drive.resetPose(pose), //Reset Pose Command
      () -> drive.getRobotRelativeSpeeds(), //Robot Relative Speed Supplier
      speeds -> drive.runVelocity(speeds), //Output Command
      new PPHolonomicDriveController( //Holonomic Drive Controller Used by PathPlanner
        new PIDConstants(5.0, 0.0, 0.0), //Translation PID Constants
        new PIDConstants(5.0, 0.0, 0.0), //Rotational PID Constants
        Constants.ROBOT_PERIOD), //PID Period
      Constants.PATHPLANNER_CONFIG,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      drive);
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
      new DriveWithJoysticks(
        drive, () -> -driver.getLeftX(), () -> driver.getLeftY(), () -> -driver.getRightX()));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Path");
  }
}
