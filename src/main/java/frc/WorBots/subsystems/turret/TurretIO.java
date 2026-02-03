package frc.WorBots.subsystems.turret;

import frc.WorBots.subsystems.turret.Turret.turretControlMode;

public interface TurretIO {

    public class TurretIOInputs{
      public double turretPosition;
      public double goalPosition;
      turretControlMode controlMode = turretControlMode.Disabled; 

    }

  public void setVoltage(double volts) ;

  public void setPosition(double turretPosition);

  public void resetZero(TurretIOInputs inputs, double position);

  public void updateInputs(TurretIOInputs inputs);

  public void setControlMode(Turret turretControlMode);

  public double getPosition();


}
