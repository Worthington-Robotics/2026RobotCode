package frc.WorBots.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.lights.Lights.LightEffects;
import frc.WorBots.subsystems.spindexer.Spindexer;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.vision.apriltags.TagVision;
import frc.WorBots.util.UtilCommands;

//TODO implement check vision test and implement lights tests
/** A class containing commands to be used to perform pit tests. */
public class PitTest {
  // The amount to wait in between steps
  private double wait = 1.0;

  /**
   * Runs a full pit test; testing all subsystems.
   * 
   * @param drive              The drive to test
   * @param superstructure     The superstructure to test
   * @param intake             The intake to test
   * @param spindexer          The spindexerto test
   * @param vision             The vision to test(not currently implemented)
   * @param lights             The lights to test (not currently implemented)
   * @param nextButtonSupplier The button used to signal we can move to the next
   *                           test
   */
  public Command fullPitTest(Drive drive, Superstructure superstructure, Intake intake, Spindexer spindexer,
      TagVision vision, Lights lights) {
    return Commands.sequence(
        // Test drive
        testDrive(drive),
        // Test intake
        testIntake(intake),
        // Test vision
        testVision(vision),
        // Test shooter and spindexer and take a test shot
        testShooterAndSpindexer(superstructure, spindexer),
        // Test turret
        testTurret(superstructure));
  }

  /**
   * Tests the drive
   * 
   * @param drive              The drive to test
   * @param nextButtonSupplier The button used to signal we can move to the next
   *                           test
   */
  public Command testDrive(Drive drive) {
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
  public Command testIntake(Intake intake) {
    return Commands.sequence(
        // Test Intake
        intake.runOnce(() -> intake.extend()).withTimeout(1.5),
        Commands.waitSeconds(wait),
        intake.runOnce(() -> intake.setVoltsIntake(Constants.IntakeConstants.INTAKE_VOLTAGE)).withTimeout(1.0),
        Commands.waitSeconds(3),
        intake.runOnce(() -> intake.setVoltsIntake(0)).withTimeout(1.0),
        Commands.waitSeconds(wait),
        intake.runOnce(() -> intake.agitate()).withTimeout(1.5),
        Commands.waitSeconds(wait),
        intake.runOnce(() -> intake.extend()).withTimeout(1.5));
  }

  /**
   * Tests the shooter and the spindexer and takes a test shot
   * 
   * @param superstructure     The superstructure containing the shooter to test
   * @param spindexer          The spindexer to test
   * @param nextButtonSupplier The button used to signal we can move to the next
   *                           test
   */
  public Command testShooterAndSpindexer(Superstructure superstructure, Spindexer spindexer) {
    return Commands.sequence(
        // Test Shooter
        superstructure.runOnce(() -> superstructure.setHoodPose(Math.PI / 3)).withTimeout(1.0),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> superstructure.hoodInPosition()),
        superstructure.runOnce(() -> superstructure.setHoodPose(0)).withTimeout(1.0),
        Commands.waitSeconds(wait),
        Commands.waitUntil(() -> superstructure.hoodInPosition()),
        superstructure.runOnce(() -> superstructure.setFlyWheelSpeed(50)).withTimeout(3.0),
        Commands.waitSeconds(2.0),

        // Test Spindexer and take a test shot
        spindexer.runOnce(() -> spindexer.runSpindexerVoltage(5)).withTimeout(2.0),
        Commands.waitSeconds(wait),
        spindexer.runOnce(() -> spindexer.stopSpindexer()).withTimeout(1.0),
        superstructure.runOnce(() -> superstructure.setFlyWheelSpeed(0)).withTimeout(3.0),
        superstructure.runOnce(() -> superstructure.setHoodPose(0)).withTimeout(3.0),
        Commands.waitSeconds(wait));
  }

  /**
   * A command used to test the turret
   * 
   * @param superstructure     The superstructure containing the turret to test
   * @param nextButtonSupplier The button used to signal we can move to the next
   *                           test
   */
  public Command testTurret(Superstructure superstructure) {
    return Commands.sequence(
        // Test turret
        superstructure.runOnce(() -> superstructure.setTurretPose(Units.degreesToRadians(160))).withTimeout(1.0),
        Commands.waitUntil(() -> superstructure.turretReady()),
        Commands.waitSeconds(wait),
        Commands.runOnce(() -> Lights.getInstance().runEffect(LightEffects.superStar)),
        superstructure.runOnce(() -> superstructure.setTurretPose(-Units.degreesToRadians(160))).withTimeout(1.0),
        Commands.waitUntil(() -> superstructure.turretReady()),
        Commands.waitSeconds(wait),
        superstructure.runOnce(() -> superstructure.setTurretPose(Units.degreesToRadians(180))).withTimeout(1.0),
        Commands.waitUntil(() -> superstructure.turretReady()),
        Commands.waitSeconds(wait),
        superstructure.runOnce(() -> superstructure.setTurretPose(0)).withTimeout(1.0),
        Commands.waitUntil(() -> superstructure.turretReady()),
        Commands.waitSeconds(wait));

  }

  public Command testVision(TagVision vision) {
    return Commands.waitUntil(() -> vision.canSeeTag());
  }
}
