// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.WorBots;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.WorBots.commands.ClimberTestCommands;
import frc.WorBots.commands.DriveWithJoysticks;
import frc.WorBots.subsystems.climber.Climber;
import frc.WorBots.subsystems.climber.ClimberIOSim;
import frc.WorBots.subsystems.climber.ClimberIOTalon;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.drive.GyroIOPigeon2;
import frc.WorBots.subsystems.drive.GyroIOSim;
import frc.WorBots.subsystems.drive.ModuleIOSim;
import frc.WorBots.subsystems.drive.ModuleIOTalon;
import frc.WorBots.util.control.DriveController;

public class RobotContainer {
  //Subsystems
  public final Drive drive;
  public final Climber climber;

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
      climber = new Climber(new ClimberIOTalon());
    } else {
      drive = new Drive(
        new GyroIOSim(), 
        new ModuleIOSim(0), 
        new ModuleIOSim(1), 
        new ModuleIOSim(2), 
        new ModuleIOSim(3));
      climber = new Climber(new ClimberIOSim());
    }

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
      new DriveWithJoysticks(
        drive, () -> -driver.getLeftX(), () -> driver.getLeftY(), () -> -driver.getRightX()));

    driver.a().onTrue(new ClimberTestCommands().climb(climber));

    driver.b().whileTrue(new ClimberTestCommands().voltClimb(climber, 10));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
