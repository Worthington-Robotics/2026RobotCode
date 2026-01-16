package frc.WorBots;

public class Constants {
  public static final double ROBOT_PERIOD = 50.0;
  public static final boolean IS_COMP = false;
  public static final boolean getSim(){
    return Robot.isSimulation();
  }

  //Robot Constants
  public static final double ROBOT_LENGTH = 0.0;
  public static final double ROBOT_WIDTH = 0.0;

  //Drive Constants
  public static final double DRIVE_MULTIPLIER = 1.0;
  public static final double DRIVE_GEAR_RATIO = 5.36;
  public static final double TURN_GEAR_RATIO = 18.75;
    /**
 * The minimum speed percentage of the maximum that can be set before angle changes are ignored
 */
  public static final double ANTI_JITTER_THRESHOLD = 0.005;

}
