package frc.WorBots.subsystems.drive;

import edu.wpi.first.math.util.Units;

/* Mode for the drivetrain when it stops, enabling things like auto-lock to prevent hits */
public enum StopMode {
  None(new double[4]),
  BlockForwardBackward(
      new double[] {
        Units.degreesToRadians(90.0),
        Units.degreesToRadians(90.0),
        Units.degreesToRadians(90.0),
        Units.degreesToRadians(90.0)
      }),
  BlockLeftRight(new double[4]),
  BlockAll(
      new double[] {
        Units.degreesToRadians(45.0),
        Units.degreesToRadians(-45.0),
        Units.degreesToRadians(-45.0),
        Units.degreesToRadians(45.0)
      }),
  ;

  public final double[] moduleAngles;

  private StopMode(double[] moduleAngles) {
    this.moduleAngles = moduleAngles;
  }
}
