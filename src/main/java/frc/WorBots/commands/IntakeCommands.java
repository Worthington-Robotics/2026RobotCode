package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.WorBots.subsystems.intake.*;

public class IntakeCommands {
    public Command intake(Intake intake) {
        // Does not end on its own, unless something else cancels it.
        return intake.startEnd(() -> {
            intake.setVoltsIntake(5);
        }, () -> {
            intake.setVoltsIntake(0);
        });

    }

    public Command extend(Intake extend){
        return extend.runOnce(() -> {extend.setVoltsExtending(5);});
    }

    public Command retract(Intake extend) {
        return extend.runOnce(()-> {extend.setVoltsExtending(-5);});
    }
}
