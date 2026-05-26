package frc.WorBots.subsystems.intake;

/**
 * Stores the intake setpoints
 */
public enum IntakePoses {
  EXTENDED(0.12),
  RETRACTED(1.4),
  HALF(1.2);

  public final double pose;

  /**
   * Stores the intake setpoints
   */
  private IntakePoses(double pose) {
    this.pose = pose;
  }

  public double get() {
    return pose;
  }
}
