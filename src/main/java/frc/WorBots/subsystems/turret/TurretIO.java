package frc.WorBots.subsystems.turret;

public interface TurretIO {

    public class TurretIOInputs{
      public double turretAbsAngle;
      public double turretRelAngle;
      public double turretFusedAngle;
      public double goalAngle;
      public boolean absEncoderConnected = false;
      public boolean shouldReadAbsEncoder = false;
    }

  public void setVoltage(double volts);

  public void updateInputs(TurretIOInputs inputs);

  public void resetOffset();

  //public void resetZero(){}
}
