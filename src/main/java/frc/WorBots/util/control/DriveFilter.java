// Copyright (c) 2024 FRC 4145
// https://github.com/Worthington-Robotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.WorBots.util.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.WorBots.util.debug.NTLogger;
import frc.WorBots.util.math.GeneralMath;
import frc.WorBots.util.math.GeomUtil;

public class DriveFilter {
  // Max acceleration filters
  private final DerivativeFilter xFilter;
  private final DerivativeFilter yFilter;
  private final DerivativeFilter thetaFilter;

  private double baseMaxVelocity;
  private double baseMaxAcceleration;
  private double baseMaxRotationalVelocity;
  private double baseMaxRotationalAcceleration;

  private double maxVelocity;
  private double maxAcceleration;
  private double maxRotationalVelocity;
  private double maxRotationalAcceleration;

  private ChassisSpeeds goalSpeeds = new ChassisSpeeds();

  /**
   * Creates a new drive filter with parameters
   *
   * @param maxVelocity The maximum velocity of the drivebase in meters per second
   * @param maxAcceleration The maximum acceleration of the drivebase in meters per second squared
   * @param maxRotationalVelocity The maximum rotational velocity of the drivebase in radians per
   *     second
   * @param maxRotationalAcceleration The maximum rotational acceleration of the drivebase in
   *     radians per second squared
   */
  public DriveFilter(
      double maxVelocity,
      double maxAcceleration,
      double maxRotationalVelocity,
      double maxRotationalAcceleration) {
    this.baseMaxVelocity = maxVelocity;
    this.baseMaxAcceleration = maxAcceleration;
    this.baseMaxRotationalVelocity = maxRotationalVelocity;
    this.baseMaxRotationalAcceleration = maxRotationalAcceleration;

    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.maxRotationalVelocity = maxRotationalVelocity;
    this.maxRotationalAcceleration = maxRotationalAcceleration;

    this.xFilter = new DerivativeFilter(maxAcceleration);
    this.yFilter = new DerivativeFilter(maxAcceleration);
    this.thetaFilter = new DerivativeFilter(maxRotationalAcceleration);
  }

  /**
   * Sets the goal speeds of the filter
   *
   * @param speeds The field-relative goal speeds
   */
  public void setGoal(ChassisSpeeds speeds) {
    goalSpeeds = speeds;
  }

  /**
   * Calculates the next value of the filter given a setpoint ChassisSpeeds
   *
   * @return The new setpoint of the drivebase, field-relative
   */
  public ChassisSpeeds calculate() {
    // Deep copy the goal speeds since we will be modifying it
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            goalSpeeds.vxMetersPerSecond,
            goalSpeeds.vyMetersPerSecond,
            goalSpeeds.omegaRadiansPerSecond);

    // Limit velocity
    final double currentVelocity = GeomUtil.getChassisSpeedsMagnitude(speeds);
    final double newVelocity = MathUtil.clamp(currentVelocity, -maxVelocity, maxVelocity);
    if (currentVelocity != 0.0) {
      speeds.vxMetersPerSecond = speeds.vxMetersPerSecond / currentVelocity * newVelocity;
      speeds.vyMetersPerSecond = speeds.vyMetersPerSecond / currentVelocity * newVelocity;
    }
    speeds.omegaRadiansPerSecond =
        MathUtil.clamp(speeds.omegaRadiansPerSecond, -maxRotationalVelocity, maxRotationalVelocity);

    // Update filters
    xFilter.setMaxDerivative(maxAcceleration);
    yFilter.setMaxDerivative(maxAcceleration);
    thetaFilter.setMaxDerivative(maxRotationalAcceleration);

    NTLogger.putNumber("Debug", "Max Accel", maxAcceleration);
    // Limit acceleration
    NTLogger.putNumber("Debug", "vx", speeds.vxMetersPerSecond);
    speeds.vxMetersPerSecond = xFilter.calculate(speeds.vxMetersPerSecond);
    NTLogger.putNumber("Debug", "vxAcell", speeds.vxMetersPerSecond);
    speeds.vyMetersPerSecond = yFilter.calculate(speeds.vyMetersPerSecond);
    speeds.omegaRadiansPerSecond = thetaFilter.calculate(speeds.omegaRadiansPerSecond);

    return speeds;
  }

  public void reset() {
    xFilter.reset();
    yFilter.reset();
    thetaFilter.reset();
  }

  /**
   * Updates the parameters of the drive filter
   *
   * @param maxVelocity The maximum velocity of the drivebase in meters per second
   * @param maxAcceleration The maximum acceleration of the drivebase in meters per second squared
   * @param maxRotationalVelocity The maximum rotational velocity of the drivebase in radians per
   *     second
   * @param maxRotationalAcceleration The maximum rotational acceleration of the drivebase in
   *     radians per second squared
   */
  public void setLimits(
      double maxVelocity,
      double maxAcceleration,
      double maxRotationalVelocity,
      double maxRotationalAcceleration) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    this.maxRotationalVelocity = maxRotationalVelocity;
    this.maxRotationalAcceleration = maxRotationalAcceleration;
  }

  /**
   * Updates the filter limits based on the height of an elevator
   *
   * @param elevatorPosition The relative position of the elevator, with 0 being lowest height and 1
   *     being highest
   * @param maxVelocity The maximum velocity of the drivebase in meters per second when the elevator
   *     is all the way up
   * @param maxAcceleration The maximum acceleration of the drivebase in meters per second squared
   *     when the elevator is all the way up
   * @param maxRotationalVelocity The maximum rotational velocity of the drivebase in radians per
   *     second when the elevator is all the way up
   * @param maxRotationalAcceleration The maximum rotational acceleration of the drivebase in
   *     radians per second squared when the elevator is all the way up
   */
  public void limitBasedOnElevator(
      double elevatorPosition,
      double maxVelocity,
      double maxAcceleration,
      double maxRotationalVelocity,
      double maxRotationalAcceleration) {
    // Fix for the scale functions
    // elevatorPosition = 1.0 - elevatorPosition;
    // SmartDashboard.putNumber("Elevator Position", elevatorPosition);

    final double newMaxVelocity = GeneralMath.scale(elevatorPosition, baseMaxVelocity, maxVelocity);
    final double newMaxAcceleration =
        GeneralMath.scale(elevatorPosition, baseMaxAcceleration, maxAcceleration);
    final double newMaxRotationalVelocity =
        GeneralMath.scale(elevatorPosition, baseMaxRotationalVelocity, maxRotationalVelocity);
    final double newMaxRotationalAcceleration =
        GeneralMath.scale(
            elevatorPosition, baseMaxRotationalAcceleration, maxRotationalAcceleration);

    setLimits(
        newMaxVelocity, newMaxAcceleration, newMaxRotationalVelocity, newMaxRotationalAcceleration);
  }

  public double getMaxVelocity() {
    return maxVelocity;
  }

  public double getMaxAcceleration() {
    return maxAcceleration;
  }

  public double getMaxRotationalVelocity() {
    return maxRotationalVelocity;
  }

  public double getMaxRotationalAcceleration() {
    return maxRotationalAcceleration;
  }

  public ChassisSpeeds getLastAcceleration() {
    return new ChassisSpeeds(xFilter.getLastDerivative(), yFilter.getLastDerivative(), thetaFilter.getLastDerivative());
  }
}
