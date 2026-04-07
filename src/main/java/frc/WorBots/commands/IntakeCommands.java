package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.intake.*;

/** A class containing commands to command the intake */
public class IntakeCommands {

  /***
   * Command to run intake, runs on its own until something cancels it
   * 
   * @param intake An intake object
   ***/
  public Command intake(Intake intake) {
    return intake.startEnd(() -> {
      intake.setVoltsIntake(Constants.IntakeConstants.INTAKE_VOLTAGE);
    }, () -> {
      intake.setVoltsIntake(0);
    });

  }

  /***
   * Command to run intake, runs on its own until something cancels it
   * 
   * @param intake An intake object
   ***/
  public Command spit(Intake intake) {
    return intake.startEnd(() -> {
      intake.setVoltsIntake(-Constants.IntakeConstants.INTAKE_VOLTAGE);
    }, () -> {
      intake.setVoltsIntake(0);
    });
  }

  /***
   * Command to extend the intake, works by supplying voltage to the extending
   * motor
   * 
   * @param extend An intake object
   */
  public Command extend(Intake extend) {
    return extend.runOnce(() -> {
      extend.extend();
    });
  }

  /***
   * Command to retract the intake, works by supplying voltage to the extending
   * motor in the opposite direction.
   * 
   * @param intake The intake to retract
   */
  public Command retract(Intake intake) {
    return intake.runOnce(() -> {
      intake.retract();
    });
  }

  /**
   * Moves the intake to a half extended position
   * 
   * @param intake The intake to agitate with
   */
  public Command agitate(Intake intake) {
    return Commands.runOnce(() -> {
      intake.agitate();
    });
  }

  /**
   * Toggles the intake position between agitate and extend
   * 
   * @param intake The intake to command
   */
  public Command togglePose(Intake intake) {
    return intake.runOnce(() -> {
      if (intake.isExtended()) {
        intake.agitate();
      } else {
        intake.extend();
      }
    });
  }

  public Command pulse(Intake intake){
    return Commands.runOnce(() -> intake.pulse(Constants.IntakeConstants.INTAKE_VOLTAGE));
  }

  public Command pulseTeleop(Intake intake){
    return intake.startEnd(() -> intake.pulse(Constants.IntakeConstants.INTAKE_VOLTAGE), () -> intake.setVoltsIntake(0));
  }
}