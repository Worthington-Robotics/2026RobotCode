package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.spindexer.Spindexer;

/** A command to run the spindexer */
public class RunSpindexer extends Command {
  private final Spindexer spin;
  private double voltage = 0;

  /**
   * A command to run the spindexer.
   * 
   * @param spin    The spindexer to run
   * @param voltage The voltage to run the spindexer at
   */
  public RunSpindexer(Spindexer spin, double voltage) {
    addRequirements(spin);
    this.spin = spin;
    this.voltage = voltage;
  }

  @Override
  public void initialize() {
    spin.runSpindexerVoltage(voltage);
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interupted) {
    spin.stopSpindexer();
  }
}
