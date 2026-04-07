package frc.WorBots.commands.pathPlannerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.spindexer.Spindexer;

public class StopShooting extends Command{
  Spindexer spin;

  public StopShooting(Spindexer spin){
    this.spin = spin;
  }

  @Override
  public void initialize(){
    spin.stopSpindexer();
  }
}
