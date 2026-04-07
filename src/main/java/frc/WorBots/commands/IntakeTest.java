package frc.WorBots.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.util.debug.TunableDouble;

/** Contains commands to test the intake */
public class IntakeTest {

  public TunableDouble voltage = new TunableDouble("Tuning", "Intake", "Voltage Mult");

  public Command retractWithVoltStick(Intake intake, Supplier<Double> volts){
    return intake.runOnce( () -> { intake.setVoltsExtending(-volts.get() * 4); });
  }

  public Command retractWithVolt(Intake intake, double volts){
    return intake.runOnce( () -> {intake.setVoltsExtending(-volts);});
  }

  public Command extendWithVoltStick(Intake intake, Supplier<Double> volts){
    return intake.runOnce(() -> {intake.setVoltsExtending(volts.get() * 4);});
  }

  public Command extendWithVolt(Intake intake, double volts){
    return intake.runOnce( () -> {intake.setVoltsExtending(volts);});
  }

  public Command intakeWithVoltStick(Intake intake, Supplier<Double> volts){
    return intake.runOnce( () -> {intake.setVoltsIntake(volts.get() * voltage.get());});
  }

  public Command intakeWithVolt(Intake intake, double volts){
    return intake.runOnce( () -> {intake.setVoltsIntake(volts);});
  }
  
}
