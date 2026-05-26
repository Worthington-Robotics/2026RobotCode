package frc.WorBots.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.WorBots.Constants;
import frc.WorBots.util.debug.StatusPage;
import frc.WorBots.util.energy.PowerLogger.SubsystemLog;
import frc.WorBots.util.math.GeneralMath;
import java.util.ArrayList;

public class Module {
  private final int index;
  private final ModuleIO io;
  private SwerveModuleState lastSetpoint = new SwerveModuleState();

  /**
   * This module represents one swerve module, which includes a turn motor, and
   * drive motor.
   *
   * @param io    The module IO to be ran with.
   * @param index The index of the module from 0-3.
   */
  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
  }

  public void periodic() {
    io.updateInputs();
    io.getInputs().drive.publish();
    io.getInputs().turn.publish();
    StatusPage.reportStatus(StatusPage.SMODULE_PREFIX + index, io.getInputs().isConnected);
  }

  /**
   * Calculates and sets the current motors to the provided state. It is
   * recommended to first call
   * optimizeState() on the setpoint in order to have optimal control
   *
   * @param state The desired state.
   * @param force Whether or not to force the state, even if the drive velocity is
   *              zero.
   */
  public void runState(SwerveModuleState state, boolean force) {
    // Perform anti-jitter to prevent module rotations for very small motions
    if (!force && Math.abs(state.speedMetersPerSecond) < 4.5 * Constants.DriveConstants.ANTI_JITTER_THRESHOLD) {
      state.angle = lastSetpoint.angle;
      state.speedMetersPerSecond = 0.0;
    }

    io.setAngle(state.angle.getRadians());
    io.setDriveSpeed(state.speedMetersPerSecond);

    lastSetpoint = state;
  }

  /**
   * Calculates the optimized version of a setpoint state
   *
   * @param state The setpoint state to optimize
   * @return The optimized state
   */
  public SwerveModuleState optimizeState(SwerveModuleState state) {
    // Get the wrapping absolute angle delta
    final Rotation2d delta = GeneralMath.wrappingAngleDifference(state.angle, getAngle());
    // Turns greater than 90 degrees can just have the module flip its direction
    if (Math.abs(delta.getDegrees()) > 90.0) {
      state = new SwerveModuleState(
          -state.speedMetersPerSecond, state.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    }

    // Stray module correction
    state.speedMetersPerSecond *= Math.cos(io.getInputs().turnPositionErrorRad);

    return state;
  }

  /**
   * Gets the current state of the module.
   *
   * @return The state.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Sets both motors to 0 volts */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /**
   * Gets the current position of the drive motor.
   *
   * @return The position in meters.
   */
  public double getPositionMeters() {
    return io.getInputs().driveDistanceMeters;
  }

  /**
   * Gets the current position of the drive motor in radians
   *
   * @return The position in radians
   */
  public double getPositionRads() {
    return io.getInputs().drive.positionRads;
  }

  /**
   * Gets the current velocity of the module.
   *
   * @return The velocity in meters per second.
   */
  public double getVelocityMetersPerSec() {
    return io.getInputs().driveVelocityMetersPerSec;
  }

  /**
   * Gets the current angle of the module.
   *
   * @return The angle as a rotation.
   */
  public Rotation2d getAngle() {
    return new Rotation2d(io.getInputs().turnAbsolutePositionRad);
  }

  public ArrayList<Double> getDrivePositionUpdates() {
    return io.getInputs().driveDistanceUpdates;
  }

  public ArrayList<Double> getTurnPositionUpdates() {
    return io.getInputs().turnPositionUpdates;
  }

  public SubsystemLog getModulePowerLog(){return io.getMotorReports();}
}
