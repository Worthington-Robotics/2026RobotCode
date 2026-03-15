package frc.WorBots.subsystems.intake;

public enum IntakePoses {
  EXTENDED(0.06),
  RETRACTED(1.4),
  HALF(.8); // Was .715

  public final double pose;

  private IntakePoses(double pose) {
    this.pose = pose;
  }

  public double get() {
    return pose;
  }
}
