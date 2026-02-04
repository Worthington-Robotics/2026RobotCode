package frc.WorBots.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.RobotContainer;
import frc.WorBots.subsystems.drive.Drive;

public class FollowTrajectory extends Command {
  private final Drive drive;
  private final Trajectory trajectory;

  private double startTimestamp = 0;
  HolonomicDriveController controller = null;

  public FollowTrajectory(Drive drive, Trajectory trajectory){
    this.drive = drive;
    this.trajectory = trajectory;
  }

  @Override
  public void initialize(){
    startTimestamp = Timer.getTimestamp();
    controller = new HolonomicDriveController(
    new PIDController(0, 0, 0), //TODO: Add PID Values
    new PIDController(0, 0, 0),
    new ProfiledPIDController(0, 0, 0, null)
    );
  }

  /*
   * Uses the given trajectory to find values needed to
   * move the robot along the trajectory and assigns them
   * to the robot.
   */
  @Override
  public void execute(){
    double timeDelta = Timer.getTimestamp() - startTimestamp;
    Trajectory.State goal = trajectory.sample(timeDelta);

    // I am 70% sure that goal.poseMeters.getRotation gives the correct rotation but could still be wrong
    ChassisSpeeds adjustedSpeeds = controller.calculate(drive.getPose(), goal, goal.poseMeters.getRotation());
    RobotContainer.driveController.drive(drive, adjustedSpeeds);
    
  }
}

