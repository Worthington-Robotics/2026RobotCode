package frc.WorBots.subsystems.turret;

public interface TurretIO {

    

  public default void setVoltage(double volts) {}
//TODO change PivotIOInputs to TurretIOInputs
  public default void resetZero(PivotIOInputs inputs, double position) {}

  public default void updateInputs(PivotIOInputs inputs) {}
}
