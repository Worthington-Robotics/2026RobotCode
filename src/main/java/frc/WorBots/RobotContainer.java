// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.WorBots;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.WorBots.commands.ConditionalFireCommand;
import frc.WorBots.commands.DriveCommands;
import frc.WorBots.commands.DriveWithJoysticks;
import frc.WorBots.commands.HoodControlFudgeCommand;
import frc.WorBots.commands.IntakeCommands;
import frc.WorBots.commands.pathPlannerCommands.IntakeExtendNoRequirements;
import frc.WorBots.commands.pathPlannerCommands.PathplannerIntakeCommands;
import frc.WorBots.commands.PitTest;
import frc.WorBots.commands.StartAutoAim;
import frc.WorBots.commands.ShooterCommands;
import frc.WorBots.commands.ShotControlFudgeCommand;
import frc.WorBots.auto.AutoSelector;
import frc.WorBots.commands.RunSpindexer;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.drive.GyroIOPigeon2;
import frc.WorBots.subsystems.drive.GyroIOSim;
import frc.WorBots.subsystems.drive.ModuleIOSim;
import frc.WorBots.subsystems.drive.ModuleIOTalon;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.intake.IntakeIOSim;
import frc.WorBots.subsystems.intake.IntakeIOTalon;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.spindexer.Spindexer;
import frc.WorBots.subsystems.spindexer.SpindexerIOSim;
import frc.WorBots.subsystems.spindexer.SpindexerIOTalon;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.shooter.ShooterIOSim;
import frc.WorBots.subsystems.superstructure.shooter.ShooterIOTalon;
import frc.WorBots.subsystems.superstructure.turret.TurretIOSim;
import frc.WorBots.subsystems.superstructure.turret.TurretIOTalon;
import frc.WorBots.subsystems.vision.apriltags.TagVision;
import frc.WorBots.subsystems.vision.apriltags.TagVisionIONew;
import frc.WorBots.util.FireController;
import frc.WorBots.util.control.DriveController;
import frc.WorBots.util.debug.StatusPage;

public class RobotContainer {
  // Subsystems
  public final Drive drive;
  public final Spindexer spin;
  public final Superstructure superstructure;
  public final Intake intake;
  public final TagVision vision;

  // Joysticks
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController operator = new CommandXboxController(1);

  // Drive Controller
  public static final DriveController driveController = new DriveController();

  /** Whether proper autos with a valid alliance have been generated */
  public static boolean validAutosGenerated = false;

  private boolean ranAuto = false;

  public static Optional<Alliance> allianceUsedForAutos = Optional.empty();

  // Auto Selector
  private AutoSelector selector;

  public RobotContainer() {
    // setup Subsystems
    if (!Constants.getSim()) {
      drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalon(0),
          new ModuleIOTalon(1),
          new ModuleIOTalon(2),
          new ModuleIOTalon(3));
      spin = new Spindexer(new SpindexerIOTalon());
      superstructure = new Superstructure(new ShooterIOTalon(), new TurretIOTalon(), drive);
      intake = new Intake(new IntakeIOTalon());
    } else {
      drive = new Drive(
          new GyroIOSim(),
          new ModuleIOSim(0),
          new ModuleIOSim(1),
          new ModuleIOSim(2),
          new ModuleIOSim(3));
      spin = new Spindexer(new SpindexerIOSim());
      superstructure = new Superstructure(new ShooterIOSim(), new TurretIOSim(), drive);
      intake = new Intake(new IntakeIOSim());
    }
    vision = new TagVision(new TagVisionIONew("left_cam"), new TagVisionIONew("right_cam"));
    vision.setDataInterfaces(drive::addVisionUpdate, () -> drive.getRotation(), () -> drive.getFieldRelativeSetpointSpeeds());

    FireController fireController = new FireController(superstructure, drive, spin);

    // TODO make the subsystems target on their own, this means this constructor
    // needs

    AutoBuilder.configure(
        () -> drive.getPose(), // Get Pose Command
        pose -> drive.resetPose(pose), // Reset Pose Command
        () -> drive.getRobotRelativeSpeeds(), // Robot Relative Speed Supplier
        speeds -> driveController.drive(drive, speeds), // Output Command
        new PPHolonomicDriveController( // Holonomic Drive Controller Used by PathPlanner
            new PIDConstants(2.4, 0.0, 0.15), // Translation PID Constants
            new PIDConstants(3.2, 0.0, 0.0), // Rotational PID Constants
            Constants.RobotConstants.ROBOT_PERIOD), // PID Period
        Constants.PathPlannerConstants.PATHPLANNER_CONFIG,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
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

  public void configureBindings() {
    configureDriverRealBindings();
    configureOperatorRealBindings();
  }

  public void configureDriverRealBindings() {
    // Configure DriveWithJoysticks
    //B activates gyro lock
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive, () -> -driver.getLeftX(), () -> driver.getLeftY(), () -> -driver.getRightX(),
            () -> {return false;}, () -> driver.x().getAsBoolean()));
    // A Key = Extend or retract intake
    // TODO: check if written correctly
    driver.rightBumper().debounce(0.02).onTrue(new IntakeCommands().togglePose(intake));
    // TODO: RT for spin intake, see if that is written correctly
    driver.rightTrigger().debounce(0.02).whileTrue(new IntakeCommands().intake(intake));
    // TODO: hood down
    driver.a().debounce(0.02).onTrue(new ShooterCommands().setHoodPose(superstructure, 0));
    // TODO: intake spit
    driver.leftBumper().debounce(0.02).whileTrue(new IntakeCommands().spit(intake));
    // Reset Heading with y button
    driver.y().debounce(0.02).onTrue(new DriveCommands().resetHeading(drive));

    driver.povUp().debounce(0.02).whileTrue(new ShooterCommands().superPass(superstructure, intake, drive, spin));

  
  }

  /*
   * Add set-pose
   * One: in front of the HP station
   * Two: in front of the hub
   */
  public void configureOperatorRealBindings() {
    // RT = Command to shoot
    operator.rightTrigger().debounce(0.02).whileTrue(new ConditionalFireCommand(drive, spin, superstructure, Constants.SpindexerConstants.SPINDEXER_VOLTAGE));
    // operator.leftTrigger().debounce(0.02).whileTrue(new ShooterCommands().forceFeedShooter(spin));
    operator.leftTrigger().debounce(0.02).whileTrue(new RunSpindexer(spin, -8)); 
    //Force feed
    operator.leftBumper().debounce(0.02).whileTrue(new ShooterCommands().forceFeedShooter(spin));
    // Climber command is going to be up Dpad
    // TODO: Add driver-assist manual disable
    // Spit intake Command
    operator.b().debounce(0.02).whileTrue(new IntakeCommands().spit(intake));

    operator.a().debounce(0.02).onTrue(new StartAutoAim(superstructure));

    operator.y().debounce(0.02).onTrue(new ShooterCommands().setHoodPose(superstructure, 0));

    //Commands to do setpoint shots: set to d pad
    operator.povUp().debounce(0.02).onTrue(new ShooterCommands().manualShot(superstructure, Constants.TurretShooterConstants.HUB_SHOT, () -> operator.x().getAsBoolean()));
    operator.povDown().debounce(0.02).onTrue(new ShooterCommands().manualShot(superstructure, Constants.TurretShooterConstants.TOWER_SHOT, () -> operator.x().getAsBoolean()));
    operator.povLeft().debounce(0.02).onTrue(new ShooterCommands().manualShot(superstructure, Constants.TurretShooterConstants.LEFT_CORNER_SHOT, () -> operator.x().getAsBoolean()));
    operator.povRight().debounce(0.02).onTrue(new ShooterCommands().manualShot(superstructure, Constants.TurretShooterConstants.RIGHT_CORNER_SHOT, () -> operator.x().getAsBoolean()));
    //Commands for fudge factors: bound to dpad while x is held
    operator.povUp().debounce(0.02).onTrue(new HoodControlFudgeCommand(superstructure, 0.005, () -> operator.x().getAsBoolean()));
    operator.povDown().debounce(0.02).onTrue(new HoodControlFudgeCommand(superstructure, -0.005, () -> operator.x().getAsBoolean()));
    operator.povLeft().debounce(0.02).onTrue(new ShotControlFudgeCommand(superstructure, -2.0, () -> operator.x().getAsBoolean()));
    operator.povRight().debounce(0.02).onTrue(new ShotControlFudgeCommand(superstructure, 2.0, () -> operator.x().getAsBoolean()));
    
  }

  public Command getAutonomousCommand() {
    if (selector == null) {
      return Commands.none();
    }

    return Commands.sequence(
        selector.getCommand());
  }

  public void checkAutos() {
    if (!validAutosGenerated) {
      if (DriverStation.getAlliance().isPresent()) {
        allianceUsedForAutos = DriverStation.getAlliance();
        registerAutos();
        validAutosGenerated = true;
        SmartDashboard.putBoolean("DB/LED 0", true);
        StatusPage.reportStatus(StatusPage.AUTOS, true);
      }
    }
    SmartDashboard.putString("DB/String 9", "FMS Says: " + DriverStation.getAlliance().toString());
  }

  private void registerAutos() {
    selector = new AutoSelector("Auto Selector 2");

    NamedCommands.registerCommand("Focus Your Power", new StartAutoAim(superstructure));
    NamedCommands.registerCommand("Sustained Fire", new ConditionalFireCommand(drive, spin, superstructure, Constants.SpindexerConstants.SPINDEXER_VOLTAGE));
    NamedCommands.registerCommand("Deploy Intake", new IntakeExtendNoRequirements(intake));
    NamedCommands.registerCommand("Annoy", new IntakeCommands().agitate(intake));
    NamedCommands.registerCommand("Mag Dump", new ShooterCommands().autoSetpointShot(superstructure, Constants.TurretShooterConstants.RIGHT_CORNER_SHOT));
    NamedCommands.registerCommand("Dunk", new ShooterCommands().manualShot(superstructure, Constants.TurretShooterConstants.HUB_SHOT, () -> {return false;}));

    NamedCommands.registerCommand("Pit Test", new PitTest().fullPitTest(drive, superstructure, intake, spin, vision, Lights.getInstance()));

    new EventTrigger("Dracarys!").whileTrue(new ConditionalFireCommand(drive, spin, superstructure, Constants.SpindexerConstants.SPINDEXER_VOLTAGE));
    new EventTrigger("Mine Mine Mine").onTrue(new PathplannerIntakeCommands().startIntakeAuto(intake));
    new EventTrigger("Dude Chill").onTrue(new PathplannerIntakeCommands().stopIntakeAuto(intake));
    new EventTrigger("Hit The Deck").whileTrue(new ShooterCommands().hoodDown(superstructure));
    new EventTrigger("Deploy Intake").onTrue(new IntakeCommands().extend(intake));
    new EventTrigger("Retract Intake").onTrue(new IntakeCommands().retract(intake));
    new EventTrigger("Sustained Fire").onTrue(new ConditionalFireCommand(drive, spin, superstructure, Constants.SpindexerConstants.SPINDEXER_VOLTAGE));
    new EventTrigger("Use the Force").onTrue(new StartAutoAim(superstructure));

    // Fetchs all of the autos from Path Planner
    List<String> autos = AutoBuilder.getAllAutoNames();

    // For each auto it checks if Its a Comp or Debug auto and modifies the list of
    // registered autos
    for (String auto : autos) {
      if (Constants.IS_COMP) {
        if (!auto.substring(0, 3).equals("COMP")) {
          continue;
        }
      }
      if (!Constants.ENABLE_DEBUG_ROUTINES) {
        if (auto.substring(0, 5).equals("DEBUG")) {
          continue;
        }
      }
      selector.addRoutine(auto, new PathPlannerAuto(auto));
    }
  }

  public void disableSubsystems() {
    drive.stop();
    intake.disable();
    superstructure.disable();
    spin.disable();
  }

  public void teleopInitSubsystems(){
    if (ranAuto){
      intake.teleopInit();
      superstructure.enableAutoAiming();
      
    }
  }

  public void ranAuto(){
    ranAuto = true;
  }
}
