package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.RobotContainer;
import frc.WorBots.subsystems.climber.Climber;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.spindexer.Spindexer;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.vision.apriltags.TagVision;
import frc.WorBots.util.UtilCommands;

//TODO implement climber, vision, and lights tests
/** A class containing commands to be used to perform pit tests. */
public class PitTest {
  // The amount to wait in between steps
  private double wait = 0.5;

  /**
   * Runs a full pit test; testing all subsystems.
   * 
   * @param drive              The drive to test
   * @param superstructure     The superstructure to test
   * @param intake             The intake to test
   * @param spindexer          The spindexerto test
   * @param climber            The climber to test (not currently implemented)
   * @param vision             The vision to test(not currently implemented)
   * @param lights             The lights to test (not currently implemented)
   * @param nextButtonSupplier The button used to signal we can move to the next
   *                           test
   */
  public Command fullPitTest(Drive drive, Superstructure superstructure, Intake intake, Spindexer spindexer,
      Climber climber, TagVision vision, Lights lights, Supplier<Boolean> nextButtonSupplier) {
    return Commands.sequence(
        // Test drive
        testDrive(drive, nextButtonSupplier),
        // Test intake
        testIntake(intake, nextButtonSupplier),
        //Test vision
        testVision(vision),
        // Test shooter and spindexer and take a test shot
        testShooterAndSpindexer(superstructure, spindexer, nextButtonSupplier),
        // Test turret
        testTurret(superstructure, nextButtonSupplier)

    // Currently not implemented due to the robot lacking a climber
    // Test climber
    // climber.runOnce(() -> climber.setPosition(new
    // Rotation2d(Constants.ClimberConstants.READY_CLIMBER_POSITION))),
    // Commands.waitUntil(() -> climber.atGoal()),
    // Commands.waitSeconds(wait),
    // Commands.waitUntil(() -> nextButtonSupplier.get()),
    // climber.runOnce(() -> climber.setPosition(new
    // Rotation2d(Constants.ClimberConstants.CLIMB_POSITION))),
    // Commands.waitUntil(() -> climber.atGoal()),
    // Commands.waitSeconds(wait),
    // Commands.waitUntil(() -> nextButtonSupplier.get()),
    // climber.runOnce(() -> climber.setPosition(new Rotation2d(0))),
    // Commands.waitUntil(() -> climber.atGoal())

    );
  }

  /**
   * Tests the drive
   * 
   * @param drive              The drive to test
   * @param nextButtonSupplier The button used to signal we can move to the next
   *                           test
   */
  public Command testDrive(Drive drive, Supplier<Boolean> nextButtonSupplier) {
    return UtilCommands.namedSequence(
        "Pit Test Drive Progress",
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(2.0, 0.0, 0.0)), drive)
            .withTimeout(1.0),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-2.0, 0.0, 0.0)), drive)
            .withTimeout(1.0),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 2.0, 0.0)), drive)
            .withTimeout(1.0),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, -2.0, 0.0)), drive)
            .withTimeout(1.0),
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 2.0)), drive)
            .withTimeout(1.0),
        Commands.runOnce(drive::stop));
  }

  /**
   * Tests the intake
   * 
   * @param intake             The intake to test
   * @param nextButtonSupplier The button used to signal we can move to the next
   *                           test
   */
  public Command testIntake(Intake intake, Supplier<Boolean> nextButtonSupplier) {
    return Commands.sequence(
        // Test Intake
        intake.runOnce(() -> intake.extend()),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> nextButtonSupplier.get()),
        intake.runOnce(() -> intake.setVoltsIntake(2)),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> nextButtonSupplier.get()),
        intake.runOnce(() -> intake.setVoltsIntake(0)),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> nextButtonSupplier.get()),
        intake.runOnce(() -> intake.retract()),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> nextButtonSupplier.get()));
  }

  /**
   * Tests the shooter and the spindexer and takes a test shot
   * 
   * @param superstructure     The superstructure containing the shooter to test
   * @param spindexer          The spindexer to test
   * @param nextButtonSupplier The button used to signal we can move to the next
   *                           test
   */
  public Command testShooterAndSpindexer(Superstructure superstructure, Spindexer spindexer,
      Supplier<Boolean> nextButtonSupplier) {
    return Commands.sequence(
        // Test Shooter
        superstructure.runOnce(() -> superstructure.setHoodPose(Math.PI / 3)),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> superstructure.hoodInPosition()),
        superstructure.runOnce(() -> superstructure.setFlyWheelSpeed(2)),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> nextButtonSupplier.get()),

        // Test Spindexer and take a test shot
        spindexer.runOnce(() -> spindexer.runSpindexerVoltage(5)),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> nextButtonSupplier.get()),
        spindexer.runOnce(() -> spindexer.stopSpindexer()),
        superstructure.runOnce(() -> superstructure.setFlyWheelSpeed(0)),
        superstructure.runOnce(() -> superstructure.setHoodPose(0)),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> nextButtonSupplier.get()));
  }

  /**
   * A command used to test the turret
   * 
   * @param superstructure     The superstructure containing the turret to test
   * @param nextButtonSupplier The button used to signal we can move to the next
   *                           test
   */
  public Command testTurret(Superstructure superstructure, Supplier<Boolean> nextButtonSupplier) {
    return Commands.sequence(
        // Test turret
        superstructure.runOnce(() -> superstructure.setTurretPose(180)),
        Commands.waitUntil(() -> superstructure.turretReady()),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> nextButtonSupplier.get()),
        superstructure.runOnce(() -> superstructure.setTurretPose(-180)),
        Commands.waitUntil(() -> superstructure.turretReady()),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> nextButtonSupplier.get()),
        superstructure.runOnce(() -> superstructure.setTurretPose(0)),
        Commands.waitUntil(() -> superstructure.turretReady()),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> nextButtonSupplier.get()));
  }

  public Command testVision(TagVision vision) {
    return Commands.waitUntil(() -> vision.canSeeTag());
  }
}
