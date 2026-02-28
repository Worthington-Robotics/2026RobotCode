package frc.WorBots.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.WorBots.subsystems.turret.Turret;
import frc.WorBots.util.debug.TunableDouble;;


public class TurretTest extends Command{
  private Turret turret;
  private TunableDouble position;

  public TurretTest(Turret turret){
    addRequirements(turret);
    this.turret = turret;
    position = new TunableDouble("Debug", "Turret", "Turret Test Position");
  }

  @Override
  public void execute(){
    turret.setPosition(Units.degreesToRadians(position.get()));
    SmartDashboard.putNumber("turret pose", Units.radiansToDegrees(turret.getPosition()));
  }

}
