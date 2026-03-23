package frc.WorBots.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.RobotContainer;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.util.math.AllianceFlipUtil;

/** The main teleop drive command. Controls the robot with joystick input */
public class DriveWithJoysticks extends Command {
  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;
  private final Supplier<Boolean> slowSupplier;
  private final Supplier<Boolean> lockGyroSupplier;
  private boolean gyroLockActive = false;
  private double gyroLockSetpoint;

  /**
   * The main teleop drive command. Controls the robot with joystick input
   * 
   * @param drive          The drive to control
   * @param leftXSupplier  Supplier providing the left joystick x value
   * @param leftYSupplier  Supplier providing the left joystick y value
   * @param rightXSupplier Supplier providing the right joystick x value
   * @param slowSupplier   A supplier that will slow down the robot when true.
   *                       Intended to be used with a button.
   * @param lockGyroSupplier When true the robot will lock to its current heading
   */
  public DriveWithJoysticks(Drive drive, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier,
      Supplier<Double> rightXSupplier, Supplier<Boolean> slowSupplier, Supplier<Boolean> lockGyroSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.slowSupplier = slowSupplier;
    this.lockGyroSupplier = lockGyroSupplier;
  }

  @Override
  public void initialize() {
    RobotContainer.driveController.reset();
  }

  @Override
  public void execute() {
    double leftX = leftXSupplier.get();
    double leftY = leftYSupplier.get();
    double rightX = rightXSupplier.get();
    if(lockGyroSupplier.get()){
      if(!gyroLockActive){
        gyroLockActive = true;
        gyroLockSetpoint = drive.getYaw().getRadians();
      }
      rightX = MathUtil.clamp((gyroLockSetpoint - drive.getYaw().getRadians()) * Constants.DriveConstants.GYRO_LOCK_KP, -0.5, 0.5);
    } else{
      gyroLockActive = false;
    }

    // Will slow the robot is slow supplier is true or we are in our alliance zone
    // if (slowSupplier.get() || (AllianceFlipUtil.shouldFlip() && drive.inRedZone())
    //     || (!AllianceFlipUtil.shouldFlip() && drive.inBlueZone())) {
    //   RobotContainer.driveController.temporarySpeedMultiplier = Optional
    //       .of(Constants.DriveConstants.DRIVE_SLOW_MULTIPLIER);
    // } else {
      RobotContainer.driveController.temporarySpeedMultiplier = Optional.empty();
    // }

    double maxSpeed = drive.getDriveMaxSpeed();
    final ChassisSpeeds speeds = RobotContainer.driveController.getSpeeds(-leftY, leftX, rightX, drive.getYaw(),
        maxSpeed, drive.getYawVelocity().getRadians());
    RobotContainer.driveController.drive(drive, speeds);
  }

  @Override
  public void end(boolean interupted) {
    drive.stop();
  }

}
