package frc.WorBots.subsystems.turret;

import frc.WorBots.util.HardwareUtils.TalonInputs;
import frc.WorBots.util.HardwareUtils.TalonInputsPositional;

public interface TurretIO {

    public class TurretIOInputs{
      public TalonInputsPositional turret = new TalonInputsPositional("Turret", "Turret Motor");
      public double turretAbsAngle;
      public double turretRelAngle;
      public double turretFusedAngle;
      public double goalAngle;
      public boolean absEncoderConnected = false;
    }

  public void setVoltage(double volts);

  public void updateInputs(TurretIOInputs inputs);

  public void resetOffset();

  //public void resetZero(){}
}
