package frc.WorBots.subsystems.intake;

public enum IntakePoses {
  EXTENDED(1.0),
  RETRACTED(0.0);

  public final double pose;

  private IntakePoses(double pose) {
    this.pose = pose;
  }
  
  public double get(){
    return pose;
  }
}

