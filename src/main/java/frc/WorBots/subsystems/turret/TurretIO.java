package frc.WorBots.subsystems.turret;

import frc.WorBots.subsystems.turret.Turret.turretControlMode;

public interface TurretIO {

    public class TurretIOInputs{
      public double turretAbsAngle;
      public double turretRelAngle;
      public double turretFusedAngle;
      public double goalAngle;
      public double debugVoltage; 
      public boolean absEncoderConnected = false;
      public boolean shouldReadAbsEncoder = false;

      turretControlMode controlMode = turretControlMode.Disabled; 

    }

  public void setVoltage(double volts);

  public void resetZero(TurretIOInputs inputs, double position);

  public void updateInputs(TurretIOInputs inputs);

  public void setControlMode(Turret turretControlMode);

  


}
