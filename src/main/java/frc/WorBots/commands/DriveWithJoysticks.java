// Copyright (c) 2026 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.RobotContainer;
import frc.WorBots.subsystems.drive.Drive;

/** The main teleop drive command. Controls the robot with joystick input */
public class DriveWithJoysticks extends Command {
  private final Drive drive;
  private final Supplier<Double> leftXSupplier;
  private final Supplier<Double> leftYSupplier;
  private final Supplier<Double> rightXSupplier;
  private final Supplier<Boolean> lockGyroSupplier;
  private boolean gyroLockActive = false;
  private double gyroLockSetpoint;
  private PIDController gyroLockPid;

  /**
   * The main teleop drive command. Controls the robot with joystick input
   * 
   * @param drive            The drive to control
   * @param leftXSupplier    Supplier providing the left joystick x value
   * @param leftYSupplier    Supplier providing the left joystick y value
   * @param rightXSupplier   Supplier providing the right joystick x value
   * @param lockGyroSupplier When true the robot will lock to its current heading
   */
  public DriveWithJoysticks(Drive drive, Supplier<Double> leftXSupplier, Supplier<Double> leftYSupplier,
      Supplier<Double> rightXSupplier, Supplier<Boolean> lockGyroSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.leftXSupplier = leftXSupplier;
    this.leftYSupplier = leftYSupplier;
    this.rightXSupplier = rightXSupplier;
    this.lockGyroSupplier = lockGyroSupplier;
    gyroLockPid = new PIDController(Constants.DriveConstants.GYRO_LOCK_KP, 0, 0);
    gyroLockPid.enableContinuousInput(-Math.PI, Math.PI);
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
    if (lockGyroSupplier.get()) {
      if (!gyroLockActive) {
        gyroLockActive = true;

        if (Math.abs(MathUtil.angleModulus(drive.getYaw().getRadians())) < Units.degreesToRadians(90)) {
          gyroLockSetpoint = 0;
        } else {
          gyroLockSetpoint = Math.PI;
        }
      }
      rightX = MathUtil
          .clamp(gyroLockPid.calculate(MathUtil.angleModulus(drive.getYaw().getRadians()), gyroLockSetpoint), -1, 1);
    } else {
      gyroLockActive = false;
    }
    RobotContainer.driveController.temporarySpeedMultiplier = Optional.empty();

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
