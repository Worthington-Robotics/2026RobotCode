package frc.WorBots.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.WorBots.Constants;
import frc.WorBots.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret {

  public final TurretIO io;
  //TODO don't use turretIOTalon in turret.java, instead just use io which will be either a Talon or Sim depending on the state
  private TurretIOInputs inputs = new TurretIOInputs();
  private turretControlMode controlMode = turretControlMode.Disabled;
  
  

  public enum turretControlMode {
    Disabled,
    Position,
    Voltage;
  }

  public Turret(TurretIO io) {
    this.io = io;
  } 

  public void disable() {
    controlMode = turretControlMode.Disabled;
  }

  public void periodic() {
    io.updateInputs(inputs);
    //TODO update voltage in periodic instead of updateInputs
  }


  public void stopTurret() {
    io.setVoltage(0.0);
  }

  public double getPosition() {
    return inputs.turretFusedAngle;
  }

  
}
