package frc.WorBots.subsystems.turret;

import frc.WorBots.subsystems.turret.TurretIO.TurretIOInputs;

public class Turret {

  public final TurretIO io;
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

  public boolean atGoal(){
    return io.atSetPoint();
  }
  
}
