package frc.WorBots.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.drive.Drive;
import frc.WorBots.subsystems.intake.Intake;
import frc.WorBots.subsystems.lights.Lights;
import frc.WorBots.subsystems.spindexer.Spindexer;
import frc.WorBots.subsystems.superstructure.Superstructure;

public class SuperPassLights extends Command {

  public SuperPassLights(){
  }

  @Override
  public void initialize(){
    Lights.getInstance().superStar(true);
  }

  @Override
  public void end(boolean interupted){
    Lights.getInstance().superStar(false);
  }
}
