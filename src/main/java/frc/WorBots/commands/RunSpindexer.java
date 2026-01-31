package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.spindexer.Spindexer;

public class RunSpindexer extends Command{
  private final Spindexer spin;
    
  public RunSpindexer(Spindexer spin){
    addRequirements(spin);
    this.spin = spin;
  }

  @Override
  public void initialize(){
    spin.runSpindexer();
  }

  @Override
  public void execute(){}

  @Override
  public void end(boolean interupted){
    spin.stopSpindexer();
  }
}
