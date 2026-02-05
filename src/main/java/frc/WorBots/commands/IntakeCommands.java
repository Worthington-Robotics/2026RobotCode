package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.intake.*;

public class IntakeCommands {

  /***
   * Command to run intake, runs on its own until something cancels it
   * 
   * @param intake An intake object
   ***/
  public Command intake(Intake intake) {
    return intake.startEnd(() -> {
      intake.setVoltsIntake(5);
    }, () -> {
      intake.setVoltsIntake(0);
    });

  }

  /***
   * Command to extend the intake, works by supplying voltage to the extending
   * motor
   * 
   * @param extend An intake object
   * @return
   */
  public Command extend(Intake extend) {
    return extend.runOnce(() -> {
      extend.setVoltsExtending(5);
    });
  }

  /***
   * Command to retract the intake, works by supplying voltage to the extending
   * motor in the
   * opposite direction
   * 
   * @param extend
   * @return
   */
  public Command retract(Intake extend) {
    return extend.runOnce(() -> {
      extend.setVoltsExtending(-5);
    });
  }
}
