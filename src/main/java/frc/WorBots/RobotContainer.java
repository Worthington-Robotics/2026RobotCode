// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.WorBots;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.WorBots.auto.AutoSelector;
import frc.WorBots.commands.DriveWithJoysticks;
import frc.WorBots.commands.ShooterTest;
import frc.WorBots.commands.TurretTest;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.drive.GyroIOPigeon2;
import frc.WorBots.subsystems.drive.GyroIOSim;
import frc.WorBots.subsystems.drive.ModuleIOSim;
import frc.WorBots.subsystems.drive.ModuleIOTalon;
import frc.WorBots.subsystems.shooter.Shooter;
import frc.WorBots.subsystems.shooter.ShooterIOSim;
import frc.WorBots.subsystems.shooter.ShooterIOTalon;
import frc.WorBots.subsystems.turret.Turret;
import frc.WorBots.subsystems.turret.TurretIOSim;
import frc.WorBots.subsystems.turret.TurretIOTalon;
import frc.WorBots.util.control.DriveController;

public class RobotContainer {
  //Subsystems
  public final Drive drive;
  public final Shooter shooter;
  public final Turret turret; 

  //Joysticks
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController operator = new CommandXboxController(1);

  //Drive Controller
  public static final DriveController driveController = new DriveController();
  
  /** Whether proper autos with a valid alliance have been generated */
  public static boolean validAutosGenerated = false;

  public static Optional<Alliance> allianceUsedForAutos = Optional.empty();

  //Auto Selector
  private AutoSelector selector;

  public RobotContainer() {
    //setup Subsystems
    if(!Constants.getSim()){
      drive = new Drive(
        new GyroIOPigeon2(), 
        new ModuleIOTalon(0), 
        new ModuleIOTalon(1), 
        new ModuleIOTalon(2), 
        new ModuleIOTalon(3));
        shooter = new Shooter(new ShooterIOTalon());
        turret = new Turret(new TurretIOTalon());
    } else {
      drive = new Drive(
        new GyroIOSim(), 
        new ModuleIOSim(0), 
        new ModuleIOSim(1), 
        new ModuleIOSim(2), 
        new ModuleIOSim(3));
      shooter = new Shooter(new ShooterIOSim()); //TODO make shooterIOSim work
      turret = new Turret(new TurretIOSim());
    }

    AutoBuilder.configure(
      () -> drive.getPose(), //Get Pose Command
      pose -> drive.resetPose(pose), //Reset Pose Command
      () -> drive.getRobotRelativeSpeeds(), //Robot Relative Speed Supplier
      speeds -> driveController.drive(drive, speeds), //Output Command
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

    registerAutos();
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
      new DriveWithJoysticks(
        drive, () -> -driver.getLeftX(), () -> driver.getLeftY(), () -> -driver.getRightX()));
    shooter.setDefaultCommand(new ShooterTest(shooter, drive, turret));
  }

  public Command getAutonomousCommand() {
    if (selector == null) {
      return Commands.none();
    }

    return selector.getCommand();
  }

  public void checkAutos() {
    if (!validAutosGenerated) {
      if (DriverStation.getAlliance().isPresent()) {
        allianceUsedForAutos = DriverStation.getAlliance();
        registerAutos();
        validAutosGenerated = true;
        SmartDashboard.putBoolean("DB/LED 0", true);
        //StatusPage.reportStatus(StatusPage.AUTOS, true);
      }
    }
    SmartDashboard.putString("DB/String 9", "FMS Says: " + DriverStation.getAlliance().toString());
  }

  private void registerAutos(){
    selector = new AutoSelector("Auto Selector 2");
    
    //Fetchs all of the autos from Path Planner
    List<String> autos = AutoBuilder.getAllAutoNames();

    //For each auto it checks if Its a Comp or Debug auto and modifies the list of registered autos
    for(String auto: autos){
      System.out.println(auto.substring(0, 6));
      if(Constants.IS_COMP){
        if(!auto.substring(0, 3).equals("COMP")){
          continue;
        }
      }
      if(!Constants.ENABLE_DEBUG_ROUTINES){
        if(auto.substring(0, 5).equals("DEBUG")){
          continue;
        }
      }
      selector.addRoutine(auto, new PathPlannerAuto(auto));
    }
  }
}
