package frc.WorBots.subsystems.turret;

public interface TurretIO {

    

  public default void setVoltage(double volts) {}

  public default void resetZero(PivotIOInputs inputs, double position) {}

  public default void updateInputs(PivotIOInputs inputs) {}
}
