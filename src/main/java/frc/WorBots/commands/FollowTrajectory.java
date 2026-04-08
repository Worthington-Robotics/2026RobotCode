package frc.WorBots.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.RobotContainer;
import frc.WorBots.subsystems.drive.Drive;

/**
 * A command used to move the robot along a trajectory. Currently not
 * implemented.
 */
public class FollowTrajectory extends Command {
  private final Drive drive;
  private final Trajectory trajectory;
  private final boolean endWithPosition;

  private double startTimestamp = 0;
  HolonomicDriveController controller;

  /**
   * A command used to move the robot along a trajectory.
   * 
   * @param drive           The drive subsystem.
   * @param trajectory      The trajectory the robot will follow.
   * @param endWithPosition Wether the position will be considered when ending the
   *                        trajectory
   */
  public FollowTrajectory(Drive drive, Trajectory trajectory, boolean endWithPosition, double maxDistance) {
    this.drive = drive;
    this.trajectory = trajectory;
    this.endWithPosition = endWithPosition;
  }

  /**
   * Gets the starting timestamp and initializes the holonomic controller.
   */
  @Override
  public void initialize() {
    startTimestamp = Timer.getFPGATimestamp();
    controller = new HolonomicDriveController(
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        new ProfiledPIDController(0, 0, 0,
            new TrapezoidProfile.Constraints(Constants.DriveConstants.DRIVE_MAX_VELOCITY,
                Constants.DriveConstants.DRIVE_MAX_ACCELERATION)));
  }

  /**
   * Uses the given trajectory to find values needed to
   * move the robot along the trajectory and assigns them
   * to the robot.
   */
  @Override
  public void execute() {

    Trajectory.State goal;
    if (getTimeDelta() >= trajectory.getTotalTimeSeconds()) {
      goal = trajectory.sample(trajectory.getTotalTimeSeconds());
    } else {
      goal = trajectory.sample(getTimeDelta());
    }

    ChassisSpeeds adjustedSpeeds = controller.calculate(drive.getPose(), goal, goal.poseMeters.getRotation());
    RobotContainer.driveController.drive(drive, adjustedSpeeds);
  }

  /**
   * Checks if the robot meets the correct requirements to stop following the
   * trajectory.
   */
  @Override
  public boolean isFinished() {
    if (endWithPosition
        && (trajectory.getTotalTimeSeconds() - getTimeDelta() <= Constants.TrajectoryConstants.MIN_TIME)) {
      Pose2d currentPose = drive.getPose();
      Translation2d goalPosition = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters.getTranslation();
      Translation2d currentPosition = currentPose.getTranslation();
      Translation2d distanceVector = goalPosition.minus(currentPosition);

      double distance = Math.abs(distanceVector.getNorm());
      if (distance <= Constants.TrajectoryConstants.MIN_DISTANCE) {
        return true;
      }
      return false;
    }

    if (Timer.getFPGATimestamp() - startTimestamp >= trajectory.getTotalTimeSeconds()) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interupted) {
    drive.stop();
  }

  /**
   * Returns the difference between current and start time.
   * 
   * @return The time delta in seconds.
   */
  Double getTimeDelta() {
    return Timer.getFPGATimestamp() - startTimestamp;
  }
}
