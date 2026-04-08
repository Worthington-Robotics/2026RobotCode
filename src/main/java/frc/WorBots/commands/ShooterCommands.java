package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.spindexer.Spindexer;
import frc.WorBots.subsystems.superstructure.Superstructure;
import frc.WorBots.subsystems.superstructure.ShotCalculator.ShootingParams;
import frc.WorBots.util.RebuiltUtils;

/** A class containing commands to control the shooter */
public class ShooterCommands {
  RebuiltUtils rebuiltUtils = new RebuiltUtils();

  /**
   * Sets the hood's position
   * 
   * @param superstructure The superstructure to use
   * @param hoodPose       The pose to set the hood to (in radians)
   */
  public Command setHoodPose(Superstructure superstructure, double hoodPose) {
    return superstructure.runOnce(() -> {
      superstructure.setHoodPose(hoodPose);
    });
  }

  /**
   * Sets the flywheel velocity
   * 
   * @param superstructure The superstructure to use
   * @param speed          The velocity to set they flywheel to (in radians per
   *                       second)
   */
  public Command setFlyWheel(Superstructure superstructure, double speed) {
    return superstructure.runOnce(() -> {
      superstructure.setFlyWheelSpeed(speed);
    });
  }

  /**
   * Feeds the shooter even if conditions aren't met
   * 
   * @param spin The spindexer to use
   */
  public Command forceFeedShooter(Spindexer spin) {
    return spin.runEnd(() -> {
      spin.setVelocity(Constants.SpindexerConstants.SPINDEXER_VELOCITY, Constants.SpindexerConstants.KICKER_VELOCITY);
    }, () -> {
      spin.stopSpindexer();
    });
  }

  public Command forceFeedShooterAuto(Spindexer spin) {
    return Commands.runOnce(() -> {
      spin.setVelocity(Constants.SpindexerConstants.SPINDEXER_VELOCITY, Constants.SpindexerConstants.KICKER_VELOCITY);
    });
  }

  /**
   * Sets the hood position based on stick input.
   * 
   * @param superstructure The superstructure to use.
   * @param stickVal       A double supplier providing the stick value
   */
  public Command commandHoodWithStick(Superstructure superstructure, Supplier<Double> stickVal) {
    return superstructure.runOnce(() -> {
      superstructure.setHoodPose(
          superstructure.getDesiredHoodPose() + (stickVal.get() * -2.5 / Constants.RobotConstants.ROBOT_FREQUENCY));
    });
  }

  /**
   * Sets the flywheel speed based on stick input
   * 
   * @param superstructure The superstructure to use.
   * @param stickVal       A double supplier providing the stick value
   */
  public Command commandFlywheelWithStick(Superstructure superstructure, Supplier<Double> stickVal) {
    return superstructure.runOnce(() -> {
      superstructure.setFlyWheelSpeed(
          superstructure.getDesiredShooterSpeed() + (stickVal.get() * -600 / Constants.RobotConstants.ROBOT_FREQUENCY));
    });
  }

  /**
   * Commands the shooter to run a manual setpoint shot.
   * 
   * @param superstructure The superstructure to use
   * @param params         The shot parameters representing the manual shot
   * @param doRunSupplier  If true the command will have no effect
   * @apiNote Sets the shooter to manual mode
   */
  public Command manualShot(Superstructure superstructure, ShootingParams params, Supplier<Boolean> doRunSupplier) {
    return Commands.runOnce(() -> {
      if (!doRunSupplier.get()) {
        superstructure.runShot(params);
      }
    });
  }

  /**
   * Sends the hood down
   * 
   * @param superstructure The superstructure containing the hood to command
   */
  public Command hoodDown(Superstructure superstructure) {
    return Commands.startEnd(() -> superstructure.setHoodDown(true), () -> superstructure.setHoodDown(false));
  }

  /**
   * Runs a setpoint shot without requirements
   * 
   * @param superstructure
   * @param params
   * @return
   */
  public Command autoSetpointShot(Superstructure superstructure, ShootingParams params) {
    return Commands.runOnce(() -> superstructure.runShot(params));
  }

  /**
   * Runs a "super pass" by passing with the shooter while ejecting balls out of
   * the intake
   * 
   * @param superstructure The superstructure to command
   * @param intake         The intake to command
   * @param drive          The drive, used for determining when the robot is ready
   *                       to fire
   * @param spin           The spindexer to command
   */
  public Command superPass(Superstructure superstructure, Intake intake, Drive drive, Spindexer spin) {
    return Commands.parallel(
        new IntakeCommands().spit(intake),
        new ConditionalFireCommand(drive, spin, superstructure, Constants.SpindexerConstants.SPINDEXER_VOLTAGE),
        new SuperPassLights());
  }
}
