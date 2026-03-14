package frc.WorBots.commands.pathPlannerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.subsystems.intake.Intake;

public class PathplannerIntakeCommands {
  public Command startIntakeAuto(Intake intake){
    return Commands.runOnce(() ->{
      intake.setVoltsIntake(8);
    });
  }

  public Command stopIntakeAuto(Intake intake){
    return Commands.runOnce(() -> {
      intake.setVoltsIntake(0);
    });
  }
}
