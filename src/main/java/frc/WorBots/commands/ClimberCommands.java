package frc.WorBots.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.climber.Climber;

/**
 * A class containing commands for use with the climber
 */
public class ClimberCommands {
  /**
   * Sets the climber to the zero position
   * 
   * @param climber The climber to set the position of
   */
  public Command resetClimber(Climber climber) {
    return climber.runOnce(() -> {
      climber.setPosition(new Rotation2d());
    });
  }

  /**
   * Sets the climber to ready position is climb lock is disabled
   * 
   * @param climber The climber to set the position of
   */
  public Command readyClimber(Climber climber) {
    return climber.runOnce(() -> {
      if (!climber.isClimbLock()) {
        climber.setPosition(Constants.ClimberConstants.READY_CLIMBER_POSITION);
      }
    });
  }

  /**
   * Sets the climber to climb position is climb lock is disabled
   * 
   * @param climber The clmber to set the position of
   */
  public Command climb(Climber climber) {
    return climber.runOnce(() -> {
      if (!climber.isClimbLock()) {
        climber.setPosition(Constants.ClimberConstants.CLIMB_POSITION);
      }
    });
  }

  /**
   * Disables climb lock, allowing the robot to climb
   * 
   * @param climber The climber to disable climb lock for
   */
  public Command setClimbLockOff(Climber climber) {
    return climber.runOnce(() -> {
      climber.setClimbLock(false);
    });
  }

  /**
   * Enables climb lock, preventing the robot from climbing
   * 
   * @param climber The climber to enable climb lock for
   */
  public Command setClimbLockOn(Climber climber) {
    return climber.runOnce(() -> {
      climber.setClimbLock(true);
    });
  }
}
